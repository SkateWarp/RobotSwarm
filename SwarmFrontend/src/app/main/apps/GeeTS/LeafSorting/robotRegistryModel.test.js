import {
    ROBOT_NAME_LIMIT,
    normalizeRobotDraft,
    robotRegistryStatus,
    validateRobotDraft,
} from "./robotRegistryModel";

describe("robotRegistryModel", () => {
    it("normalizes only fields that the registry can edit", () => {
        expect(
            normalizeRobotDraft({
                id: 8,
                name: "  tb3_0  ",
                description: "  Explorador  ",
                notes: "   ",
                status: "1",
                isPublic: 1,
                accountId: 99,
                namespace: "/tb3_0",
            })
        ).toEqual({
            name: "tb3_0",
            description: "Explorador",
            notes: null,
            status: 1,
            isPublic: true,
        });
    });

    it("requires a bounded, non-empty robot name", () => {
        expect(validateRobotDraft({ name: "tb3_0" })).toEqual({});
        expect(validateRobotDraft({ name: "   " })).toHaveProperty("name");
        expect(validateRobotDraft({ name: "R".repeat(ROBOT_NAME_LIMIT + 1) })).toHaveProperty("name");
    });

    it("presents both backend enum names and numeric values in Spanish", () => {
        expect(robotRegistryStatus({ statusDescription: "Idle", status: 0 })).toBe("Disponible");
        expect(robotRegistryStatus({ statusDescription: "Working", status: 1 })).toBe("Ocupado");
        expect(robotRegistryStatus({ statusDescription: "Disabled", status: 2 })).toBe("Deshabilitado");
        expect(robotRegistryStatus({ status: 99 })).toBe("Sin estado");
    });
});
