import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "app/services/jwtService";
import SimulationSessionService from "./SimulationSessionService";

describe("SimulationSessionService viewer lifecycle", () => {
    let mock;

    beforeEach(() => {
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("viewer-owner-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
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
