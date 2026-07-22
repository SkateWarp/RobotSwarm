import { buildTransportParameters, describeTransportResult, validateNumberField } from "./SwarmTaskPanel";

describe("task parameter validation", () => {
    it("accepts both boundaries and rejects empty or out-of-range values", () => {
        expect(validateNumberField(0.35, 0.35, 2, "Separación")).toBe("");
        expect(validateNumberField("2", 0.35, 2, "Separación")).toBe("");
        expect(validateNumberField("", 0.35, 2, "Separación")).toContain("introduce un valor");
        expect(validateNumberField(2.1, 0.35, 2, "Separación")).toContain("entre 0.35 y 2");
    });

    it("sends the visible destination and its arrival margin to ROS", () => {
        expect(buildTransportParameters("-2.5", "1.25", "0.25")).toEqual({
            target_x: -2.5,
            target_y: 1.25,
            arrival_tolerance: 0.25,
            config: {
                target_x: -2.5,
                target_y: 1.25,
                arrival_tolerance: 0.25,
                transport_planner: "grf",
            },
        });
    });
});

describe("readable collaborative transport result", () => {
    it("reports the fleet-wide search phase", () => {
        const result = describeTransportResult({
            state: "Running",
            result: {
                transport: {
                    phase: "SEARCH",
                    searching_robot_count: 7,
                },
            },
        });

        expect(result.phaseLabel).toBe("Búsqueda");
        expect(result.summary).toBe("7 robots están buscando la carga.");
        expect(result.searching).toBe(7);
    });

    it("shows discovery notices and a confirmed coordinated push", () => {
        const result = describeTransportResult({
            state: "Running",
            result: {
                transport: {
                    phase: "PUSH",
                    all_pushers_confirmed: true,
                    useful_contributor_count: 10,
                    arrival_tolerance: 0.25,
                    target_marker: {
                        model_name: "target_marker",
                        published: true,
                        synchronized: true,
                        position: { x: -2.5, y: 1.25 },
                    },
                    discovery: {
                        finder: "tb3_3",
                        object_position: { x: 1.257, y: -0.744 },
                        notified_robots: ["tb3_1", "tb3_2", "tb3_4"],
                    },
                },
            },
        });

        expect(result.phaseLabel).toBe("Empuje coordinado");
        expect(result.summary).toBe("El empuje conjunto está confirmado para 10 robots.");
        expect(result.allPushersConfirmed).toBe(true);
        expect(result.position).toBe(" (1.26, -0.74)");
        expect(result.notified).toBe(3);
        expect(result.arrivalTolerance).toBe(0.25);
        expect(result.targetMarker).toEqual({
            modelName: "target_marker",
            published: true,
            synchronized: true,
            position: { x: "-2.50", y: "1.25" },
            summary: "Destino visual confirmado en Gazebo (-2.50, 1.25)",
        });
    });

    it("keeps a failed visual marker separate from the robot task", () => {
        const result = describeTransportResult({
            state: "Running",
            result: {
                transport: {
                    phase: "SEARCH",
                    arrival_tolerance: 0.5,
                    target_marker: {
                        model_name: "target_marker",
                        published: false,
                        synchronized: false,
                        position: null,
                    },
                },
            },
        });

        expect(result.phase).toBe("SEARCH");
        expect(result.arrivalTolerance).toBe(0.5);
        expect(result.targetMarker.summary).toContain("el transporte continúa");
    });

    it("does not describe marker confirmation as pending after delivery", () => {
        const result = describeTransportResult({
            state: "Completed",
            result: {
                transport: {
                    phase: "DONE",
                    target_marker: {
                        model_name: "target_marker",
                        published: true,
                        synchronized: false,
                        position: { x: 1, y: -1 },
                    },
                },
            },
        });

        expect(result.targetMarker.summary).toBe("Destino visual enviado, sin confirmación final de Gazebo");
    });

    it("drops malformed arrival and target-marker telemetry", () => {
        const result = describeTransportResult({
            state: "Running",
            result: {
                transport: {
                    phase: "PUSH",
                    arrival_tolerance: "0.25",
                    target_marker: {
                        model_name: "../../private-model",
                        published: "yes",
                        synchronized: true,
                        position: { x: "-2.5", y: Infinity },
                    },
                },
            },
        });

        expect(result.arrivalTolerance).toBeNull();
        expect(result.targetMarker).toBeNull();
    });

    it.each(["Paused", "Cancelling", "Cancelled", "Failed"])(
        "does not describe stale movement while the task is %s",
        (state) => {
            expect(
                describeTransportResult({
                    state,
                    result: {
                        transport: {
                            phase: "PUSH",
                            all_pushers_confirmed: true,
                            useful_contributor_count: 4,
                        },
                    },
                })
            ).toBeNull();
        }
    );
});
