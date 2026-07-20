import {
    formatTaskDuration,
    formatTaskPayload,
    taskOutcomeColor,
    taskOutcomeLabel,
    taskStateColor,
    taskStateLabel,
    taskTypeLabel,
} from "./TaskRunHistoryApp";

describe("TaskRunHistoryApp helpers", () => {
    it("uses the names shown by the ROS control plane", () => {
        expect(taskTypeLabel("CollaborativeTransport")).toBe("Transporte colaborativo");
        expect(taskStateLabel("Running")).toBe("En ejecución");
        expect(taskOutcomeLabel("Succeeded")).toBe("Aprobada");
    });

    it("keeps unknown backend values visible for diagnostics", () => {
        expect(taskTypeLabel("FutureTask")).toBe("FutureTask");
        expect(taskStateLabel("FutureState")).toBe("FutureState");
        expect(taskOutcomeLabel("FutureOutcome")).toBe("FutureOutcome");
    });

    it("assigns consistent colors to terminal and transitional states", () => {
        expect(taskStateColor("Completed")).toBe("success");
        expect(taskStateColor("Failed")).toBe("error");
        expect(taskStateColor("Paused")).toBe("warning");
        expect(taskOutcomeColor("Succeeded")).toBe("success");
        expect(taskOutcomeColor("Pending")).toBe("info");
    });

    it("formats duration from the actual task timestamps", () => {
        expect(
            formatTaskDuration({
                startedAt: "2026-07-20T01:00:00Z",
                completedAt: "2026-07-20T01:02:07Z",
            })
        ).toBe("2 min 7 s");
        expect(formatTaskDuration({ createdAt: "invalid", updatedAt: "invalid" })).toBe("—");
    });

    it("renders structured parameters without hiding a missing result", () => {
        expect(formatTaskPayload({ targetX: 2 })).toContain('"targetX": 2');
        expect(formatTaskPayload(null)).toBe("Sin datos");
    });
});
