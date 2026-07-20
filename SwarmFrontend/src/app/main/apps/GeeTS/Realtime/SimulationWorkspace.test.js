/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import SimulationSessionService from "../../../../../services/SimulationSessionService";
import SimulationWorkspace, {
    mergeViewerCommandEvent,
    mergeViewerLeaseStatus,
    requestMessage,
    sendViewerControlInput,
    sessionRobotStateColor,
    sessionRobotSummary,
    viewerCommandFailureMessage,
    viewerCloseOutcomeNotice,
} from "./SimulationWorkspace";

jest.mock("../../../../../services/SimulationSessionService", () => ({
    __esModule: true,
    default: {
        getLimits: jest.fn(),
        list: jest.fn(),
        listRobots: jest.fn(),
        listTasks: jest.fn(),
        createViewerLease: jest.fn(),
        closeViewerLease: jest.fn(),
        getViewerLeaseStatus: jest.fn(),
        waitForViewerClose: jest.fn(),
        stop: jest.fn(),
        createRealtimeConnection: jest.fn(),
    },
}));
jest.mock("./HlsViewer", () => ({
    __esModule: true,
    default: () => "HLS activo",
}));
jest.mock("./WhepViewer", () => ({
    __esModule: true,
    default: () => "WHEP activo",
}));
jest.mock("./SwarmTaskPanel", () => ({
    __esModule: true,
    default: () => null,
}));

describe("simulation API problem details", () => {
    it("uses the RFC 7807 detail when the API does not return message", () => {
        const error = {
            response: {
                data: {
                    type: "https://robot.zerav.la/problems/session-capacity",
                    title: "Session unavailable",
                    detail: "The GPU worker has no free slot.",
                },
            },
        };

        expect(requestMessage(error, "fallback")).toBe("The GPU worker has no free slot.");
    });

    it("reads model validation errors before a generic problem title", () => {
        const error = {
            response: {
                data: {
                    title: "One or more validation errors occurred.",
                    errors: {
                        robotCount: ["Robot count must be between 1 and 10."],
                    },
                },
            },
        };

        expect(requestMessage(error, "fallback")).toBe("Robot count must be between 1 and 10.");
    });

    it("keeps compatibility with message and plain-text responses", () => {
        expect(requestMessage({ response: { data: { message: "Queue is full." } } }, "fallback")).toBe(
            "Queue is full."
        );
        expect(requestMessage({ response: { data: "Gateway unavailable" } }, "fallback")).toBe(
            "Gateway unavailable"
        );
    });
});

describe("viewer command lifecycle", () => {
    const lease = {
        leaseId: "014f5fc2-028f-456b-b310-f1e73c38c58f",
        token: "read-token-kept-only-in-browser",
        command: {
            id: "1c64dc41-375c-45b1-a774-95b1aa4dcb45",
            state: "Pending",
            createdAt: "2026-07-19T12:00:00Z",
        },
    };

    it("updates only the command that prepared this lease", () => {
        const unrelated = mergeViewerCommandEvent(lease, {
            eventType: "CommandUpdated",
            commandId: "82c77fdb-824b-4877-8643-057e700e7d6d",
            state: "Completed",
        });
        const matching = mergeViewerCommandEvent(lease, {
            eventType: "CommandUpdated",
            commandId: lease.command.id.toUpperCase(),
            state: "Running",
            timestamp: "2026-07-19T12:00:02Z",
        });

        expect(unrelated).toBe(lease);
        expect(matching.command.state).toBe("Running");
        expect(matching.isReady).toBe(false);
        expect(matching.token).toBe(lease.token);
    });

    it("accepts a missed completion from polling without replacing the private token", () => {
        const completed = mergeViewerLeaseStatus(lease, {
            leaseId: lease.leaseId.toUpperCase(),
            expiresAt: "2026-07-19T12:05:00Z",
            isReady: true,
            command: {
                id: lease.command.id,
                state: "Completed",
                error: null,
                updatedAt: "2026-07-19T12:00:08Z",
            },
        });

        expect(completed.command.state).toBe("Completed");
        expect(completed.isReady).toBe(true);
        expect(completed.token).toBe(lease.token);

        const stalePoll = mergeViewerLeaseStatus(completed, {
            leaseId: lease.leaseId,
            isReady: false,
            command: {
                id: lease.command.id,
                state: "Running",
                updatedAt: "2026-07-19T12:00:04Z",
            },
        });
        expect(stalePoll.command.state).toBe("Completed");
    });

    it("shows the error reported by the worker instead of a generic HLS 404", () => {
        expect(
            viewerCommandFailureMessage({
                state: "Failed",
                error: "Gazebo did not expose a display.",
            })
        ).toBe("El worker no pudo preparar el visor: Gazebo did not expose a display.");
        expect(viewerCommandFailureMessage({ state: "Cancelled" })).toContain("cancelada");
    });

    it("distinguishes a failed close from a close that is still unconfirmed", () => {
        expect(
            viewerCloseOutcomeNotice({
                state: "Failed",
                command: { error: "Publisher process is still alive." },
            })
        ).toMatchObject({
            severity: "error",
            message: expect.stringContaining("Publisher process is still alive."),
        });
        expect(viewerCloseOutcomeNotice({ state: "TimedOut" })).toMatchObject({
            severity: "warning",
            message: expect.stringContaining("aún no confirmó"),
        });
        expect(viewerCloseOutcomeNotice({ state: "Cancelled" })).toMatchObject({
            severity: "warning",
            message: expect.stringContaining("cancelado"),
        });
    });
});

describe("acknowledged viewer input", () => {
    it("uses a Hub invocation and exposes an asynchronous rejection", async () => {
        const rejection = new Error("Viewer input rate limit exceeded.");
        const connection = {
            invoke: jest.fn().mockRejectedValue(rejection),
            send: jest.fn(),
        };
        const input = { type: "releaseAll" };

        await expect(sendViewerControlInput(connection, "session-1", "lease-1", input)).rejects.toBe(
            rejection
        );
        expect(connection.invoke).toHaveBeenCalledWith("SendViewerInput", "session-1", "lease-1", input);
        expect(connection.send).not.toHaveBeenCalled();
    });
});

describe("session robot monitor", () => {
    it("summarizes operational, provisioning, and unavailable robots", () => {
        expect(
            sessionRobotSummary([
                { state: "Ready" },
                { state: "Active" },
                { state: "Provisioning" },
                { state: "Offline" },
                { state: "Failed" },
            ])
        ).toEqual({ total: 5, operational: 2, provisioning: 1, unavailable: 2 });
        expect(sessionRobotStateColor("Ready")).toBe("success");
        expect(sessionRobotStateColor("Failed")).toBe("error");
    });
});

describe("explicit viewer close", () => {
    let host;
    let connection;
    let consoleError;

    const flush = async () => {
        await act(async () => {
            await Promise.resolve();
            await Promise.resolve();
            await Promise.resolve();
        });
    };

    const button = (label) =>
        Array.from(document.querySelectorAll("button")).find((candidate) =>
            candidate.textContent.includes(label)
        );

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        connection = {
            state: "Connected",
            on: jest.fn(),
            onreconnected: jest.fn(),
            onreconnecting: jest.fn(),
            onclose: jest.fn(),
            start: jest.fn().mockResolvedValue(undefined),
            stop: jest.fn().mockResolvedValue(undefined),
            invoke: jest.fn((method) => {
                if (method === "BeginViewerControl") {
                    return Promise.resolve({ authorizedUntil: "2099-01-01T00:00:00Z" });
                }
                return Promise.resolve(undefined);
            }),
        };
        SimulationSessionService.createRealtimeConnection.mockReturnValue(connection);
        SimulationSessionService.getLimits.mockResolvedValue({ maxRobotsPerSession: 10 });
        SimulationSessionService.list.mockResolvedValue([
            {
                id: "46643be0-a7b2-44ef-95c5-30c8657c6489",
                state: "Ready",
                desiredRobotCount: 10,
                arenaVersion: "test-arena",
                computeWorkerId: "df4a00c0-2ee7-42a9-95a7-8b550bd40a63",
                computeWorkerName: "gpu-test",
                isEmergencyStopped: false,
            },
        ]);
        SimulationSessionService.listRobots.mockResolvedValue([]);
        SimulationSessionService.listTasks.mockResolvedValue([]);
        SimulationSessionService.createViewerLease.mockResolvedValue({
            leaseId: "a023236e-868c-4898-b4cb-d6361bc25945",
            token: "private-token",
            expiresAt: "2099-01-01T00:00:00Z",
            hlsUrl: "https://robot.example/private/index.m3u8",
            command: {
                id: "3c24ab34-f81b-4d0b-9822-521451696efa",
                state: "Completed",
                updatedAt: "2026-07-19T12:00:00Z",
            },
        });
        SimulationSessionService.closeViewerLease.mockResolvedValue({
            leaseId: "a023236e-868c-4898-b4cb-d6361bc25945",
        });
        SimulationSessionService.waitForViewerClose.mockResolvedValue({
            state: "Completed",
            command: { id: "stop-viewer", state: "Completed" },
        });
        SimulationSessionService.stop.mockResolvedValue({ state: "Stopping" });
    });

    afterEach(() => {
        act(() => ReactDOM.unmountComponentAtNode(host));
        host.remove();
        consoleError.mockRestore();
        jest.clearAllMocks();
        jest.useRealTimers();
    });

    it("shows Cerrar visor and clears interaction, stream, and lease state after revocation", async () => {
        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();
        await flush();

        await act(async () => {
            button("Abrir visor").click();
        });
        await flush();
        await flush();

        expect(host.textContent).toContain("HLS activo");
        expect(button("Cerrar visor")).toBeDefined();
        expect(connection.invoke).toHaveBeenCalledWith(
            "BeginViewerControl",
            "46643be0-a7b2-44ef-95c5-30c8657c6489",
            "a023236e-868c-4898-b4cb-d6361bc25945"
        );

        await act(async () => {
            button("Cerrar visor").click();
        });
        await flush();

        expect(connection.invoke).toHaveBeenCalledWith(
            "SendViewerInput",
            "46643be0-a7b2-44ef-95c5-30c8657c6489",
            "a023236e-868c-4898-b4cb-d6361bc25945",
            { type: "releaseAll" }
        );
        expect(SimulationSessionService.closeViewerLease).toHaveBeenCalledWith(
            "46643be0-a7b2-44ef-95c5-30c8657c6489",
            "a023236e-868c-4898-b4cb-d6361bc25945"
        );
        expect(host.textContent).not.toContain("HLS activo");
        expect(button("Cerrar visor")).toBeUndefined();
        expect(host.textContent).toContain("Abre el visor de tu sesión");
    });

    it("keeps an unconfirmed lease visible and succeeds when the owner retries the close", async () => {
        SimulationSessionService.closeViewerLease.mockResolvedValue({
            leaseId: "a023236e-868c-4898-b4cb-d6361bc25945",
            revokedAt: "2026-07-19T12:01:00Z",
            command: { id: "stop-viewer-1", state: "Pending" },
        });
        SimulationSessionService.waitForViewerClose
            .mockResolvedValueOnce({
                state: "TimedOut",
                command: { id: "stop-viewer-1", state: "Running" },
            })
            .mockResolvedValueOnce({
                state: "Completed",
                command: { id: "stop-viewer-1", state: "Completed" },
            });

        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();
        await flush();

        await act(async () => {
            button("Abrir visor").click();
        });
        await flush();
        expect(host.textContent).toContain("HLS activo");

        await act(async () => {
            button("Cerrar visor").click();
        });
        await flush();

        expect(host.textContent).not.toContain("HLS activo");
        expect(host.querySelector("[data-testid='viewer-close-notice']")).not.toBeNull();
        expect(host.textContent).toContain("aún no confirmó");
        expect(button("Reintentar cierre")).toBeDefined();

        await act(async () => {
            button("Reintentar cierre").click();
        });
        await flush();

        expect(SimulationSessionService.closeViewerLease).toHaveBeenCalledTimes(2);
        expect(SimulationSessionService.waitForViewerClose).toHaveBeenCalledTimes(2);
        expect(button("Reintentar cierre")).toBeUndefined();
        expect(host.textContent).toContain("Abre el visor de tu sesión");
    });

    it("shows the worker error and offers a retry after a failed close", async () => {
        SimulationSessionService.closeViewerLease.mockResolvedValue({
            leaseId: "a023236e-868c-4898-b4cb-d6361bc25945",
            revokedAt: "2026-07-19T12:01:00Z",
            command: { id: "stop-viewer-failed", state: "Pending" },
        });
        SimulationSessionService.waitForViewerClose.mockResolvedValue({
            state: "Failed",
            command: {
                id: "stop-viewer-failed",
                state: "Failed",
                error: "Publisher did not exit.",
            },
        });

        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();
        await flush();
        await act(async () => {
            button("Abrir visor").click();
        });
        await flush();
        await act(async () => {
            button("Cerrar visor").click();
        });
        await flush();

        expect(host.querySelector("[data-testid='viewer-close-notice']")).not.toBeNull();
        expect(host.textContent).toContain("Publisher did not exit.");
        expect(button("Reintentar cierre")).toBeDefined();
    });

    it("shows every runtime robot separately from the persistent inventory", async () => {
        SimulationSessionService.listRobots.mockResolvedValue([
            {
                id: "runtime-1",
                ordinal: 0,
                runtimeId: "tb3_0",
                namespace: "/tb3_0",
                role: "leader",
                state: "Active",
                updatedAt: "2026-07-19T12:00:00Z",
            },
            {
                id: "runtime-2",
                ordinal: 1,
                runtimeId: "tb3_1",
                namespace: "/tb3_1",
                role: "follower",
                state: "Offline",
                updatedAt: "2026-07-19T12:00:01Z",
            },
        ]);

        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();
        await flush();

        expect(host.querySelector("[data-testid='session-robot-monitor']")).not.toBeNull();
        expect(host.textContent).toContain("tb3_0");
        expect(host.textContent).toContain("tb3_1");
        expect(host.textContent).toContain("1/2 operativos");
        expect(host.textContent).toContain("1 robot(es) requieren atención");
    });

    it("requires confirmation before stopping and releasing the whole session", async () => {
        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();
        await flush();

        act(() => button("Detener sesión").click());
        expect(SimulationSessionService.stop).not.toHaveBeenCalled();
        expect(document.body.textContent).toContain("Esta acción no se puede deshacer");

        await act(async () => {
            button("Detener y liberar").click();
        });
        await flush();

        expect(SimulationSessionService.stop).toHaveBeenCalledWith("46643be0-a7b2-44ef-95c5-30c8657c6489");
    });

    it("retries an initial SignalR failure without requiring a page reload", async () => {
        jest.useFakeTimers();
        connection.start
            .mockRejectedValueOnce(new Error("temporary gateway failure"))
            .mockResolvedValueOnce(undefined);

        await act(async () => {
            ReactDOM.render(<SimulationWorkspace />, host);
        });
        await flush();

        expect(host.querySelector("[data-testid='realtime-status']")).not.toBeNull();
        expect(connection.start).toHaveBeenCalledTimes(1);

        await act(async () => {
            jest.advanceTimersByTime(2000);
            await Promise.resolve();
            await Promise.resolve();
        });
        await flush();

        expect(connection.start).toHaveBeenCalledTimes(2);
        expect(host.querySelector("[data-testid='realtime-status']")).toBeNull();
        expect(host.textContent).toContain("Tiempo real conectado");
        jest.useRealTimers();
    });
});
