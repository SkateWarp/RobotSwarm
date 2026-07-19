import {
    mergeViewerCommandEvent,
    mergeViewerLeaseStatus,
    requestMessage,
    sendViewerControlInput,
    viewerCommandFailureMessage,
} from "./SimulationWorkspace";

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
