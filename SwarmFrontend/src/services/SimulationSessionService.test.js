import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "app/services/jwtService";
import SimulationSessionService from "./SimulationSessionService";

describe("SimulationSessionService commands and viewer lifecycle", () => {
    let mock;

    beforeEach(() => {
        global.window = {
            crypto: { randomUUID: jest.fn(() => "test-idempotency-key") },
            setTimeout,
        };
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("viewer-owner-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
        delete global.window;
    });

    it("retries a transient command conflict with the same idempotency key", async () => {
        const sessionId = "8c0aa5bf-803b-4eae-a511-31915230a39f";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/tasks`;
        const keys = [];
        mock.onPost(endpoint).reply((request) => {
            keys.push(request.headers["Idempotency-Key"]);
            if (keys.length === 1) {
                return [
                    409,
                    {
                        code: "serialization_conflict",
                        retryable: true,
                        message: "Retry the command.",
                    },
                ];
            }
            return [202, { task: { type: "Figure", state: "Queued" } }];
        });

        await expect(
            SimulationSessionService.startTask(sessionId, "Figure", {
                formation_type: "triangle",
            })
        ).resolves.toMatchObject({ task: { type: "Figure" } });

        expect(mock.history.post).toHaveLength(2);
        expect(keys[0]).toBeTruthy();
        expect(keys[1]).toBe(keys[0]);
    });

    it("does not retry a semantic command conflict", async () => {
        const sessionId = "6cb4a95a-b190-4695-a70b-7615e21116bf";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/tasks`;
        mock.onPost(endpoint).reply(409, {
            message: "The session already has an active task.",
        });

        await expect(
            SimulationSessionService.startTask(sessionId, "Figure", {
                formation_type: "triangle",
            })
        ).rejects.toMatchObject({ response: { status: 409 } });

        expect(mock.history.post).toHaveLength(1);
    });

    it("does not turn a conflicted fleet command into a second transition", async () => {
        const sessionId = "6cb4a95a-b190-4695-a70b-7615e21116bf";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/fleet`;
        mock.onPatch(endpoint).reply(409, {
            code: "serialization_conflict",
            retryable: true,
            message: "Retry the command.",
        });

        await expect(SimulationSessionService.updateFleet(sessionId, 5)).rejects.toMatchObject({
            response: { status: 409 },
        });

        expect(mock.history.patch).toHaveLength(1);
    });

    it("revokes the exact viewer lease with the owner bearer token", async () => {
        const sessionId = "643b26c7-8ac6-4e79-ac2a-60591429a38f";
        const leaseId = "30fac27a-1468-4109-8413-c47bcefc9f46";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/viewer-lease/${leaseId}`;
        mock.onDelete(endpoint).reply(202, { sessionId, leaseId });

        await expect(SimulationSessionService.closeViewerLease(sessionId, leaseId)).resolves.toEqual({
            sessionId,
            leaseId,
            accepted: true,
        });

        const request = mock.history.delete[0];
        expect(request.headers.Authorization).toBe("Bearer viewer-owner-token");
    });

    it("waits for the durable stop command to move from queued to completed", async () => {
        const sessionId = "ca77bb90-c747-47c4-a2b7-2bd36022079e";
        const leaseId = "8da82e8a-4884-423d-9f26-543fd210909a";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/viewer-lease/${leaseId}`;
        const sleep = jest.fn().mockResolvedValue(undefined);
        mock.onGet(endpoint).replyOnce(200, {
            leaseId,
            revokedAt: "2026-07-19T12:00:00Z",
            closeCommand: { id: "stop-1", state: "Pending" },
        });
        mock.onGet(endpoint).replyOnce(200, {
            leaseId,
            revokedAt: "2026-07-19T12:00:00Z",
            closeCommand: { id: "stop-1", state: "Completed" },
        });

        await expect(
            SimulationSessionService.waitForViewerClose(sessionId, leaseId, {
                maxAttempts: 4,
                pollIntervalMs: 10,
                sleep,
            })
        ).resolves.toMatchObject({
            state: "Completed",
            command: { id: "stop-1", state: "Completed" },
            timedOut: false,
        });
        expect(sleep).toHaveBeenCalledTimes(1);
        expect(mock.history.get).toHaveLength(2);
    });

    it("returns the worker failure without consuming the remaining poll budget", async () => {
        const sessionId = "875a7c58-5655-4825-a025-f6a3d6f9a560";
        const leaseId = "0a9ae070-14cf-47c7-b6b5-6edb8cf3d305";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/viewer-lease/${leaseId}`;
        const sleep = jest.fn().mockResolvedValue(undefined);
        mock.onGet(endpoint).reply(200, {
            leaseId,
            closeCommand: {
                id: "stop-failed",
                state: "Failed",
                error: "The publisher did not exit.",
            },
        });

        await expect(
            SimulationSessionService.waitForViewerClose(sessionId, leaseId, { sleep })
        ).resolves.toMatchObject({
            state: "Failed",
            command: {
                state: "Failed",
                error: "The publisher did not exit.",
            },
            timedOut: false,
        });
        expect(sleep).not.toHaveBeenCalled();
        expect(mock.history.get).toHaveLength(1);
    });

    it("times out after a bounded number of pending status checks", async () => {
        const sessionId = "f06dd08a-26ba-4aef-9d91-a9399d767034";
        const leaseId = "a598b58b-da29-4603-8170-f230d93c1dcf";
        const endpoint = `${URL.replace(/\/+$/, "")}/api/sessions/${sessionId}/viewer-lease/${leaseId}`;
        const sleep = jest.fn().mockResolvedValue(undefined);
        mock.onGet(endpoint).reply(200, {
            leaseId,
            closeCommand: { id: "stop-slow", state: "Running" },
        });

        await expect(
            SimulationSessionService.waitForViewerClose(sessionId, leaseId, {
                maxAttempts: 3,
                pollIntervalMs: 10,
                sleep,
            })
        ).resolves.toMatchObject({
            state: "TimedOut",
            command: { id: "stop-slow", state: "Running" },
            timedOut: true,
        });
        expect(mock.history.get).toHaveLength(3);
        expect(sleep).toHaveBeenCalledTimes(2);
    });
});
