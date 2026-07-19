import { describeTransportResult, validateNumberField } from "./SwarmTaskPanel";

describe("task parameter validation", () => {
    it("accepts both boundaries and rejects empty or out-of-range values", () => {
        expect(validateNumberField(0.35, 0.35, 2, "Separación")).toBe("");
        expect(validateNumberField("2", 0.35, 2, "Separación")).toBe("");
        expect(validateNumberField("", 0.35, 2, "Separación")).toContain("introduce un valor");
        expect(validateNumberField(2.1, 0.35, 2, "Separación")).toContain("entre 0.35 y 2");
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
