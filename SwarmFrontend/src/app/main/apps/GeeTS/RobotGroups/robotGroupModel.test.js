import {
    ROBOT_GROUP_NAME_LIMIT,
    buildRobotMemberships,
    countAssignedActiveRobots,
    normalizeRobotGroupDraft,
    robotStatusLabel,
    validateRobotGroupDraft,
} from "./robotGroupModel";

describe("robotGroupModel", () => {
    it("normalizes only the editable group fields", () => {
        expect(
            normalizeRobotGroupDraft({
                id: 4,
                name: "  Exploradores  ",
                description: "  Patrulla norte  ",
                robots: [{ id: 9 }],
            })
        ).toEqual({ name: "Exploradores", description: "Patrulla norte" });
        expect(normalizeRobotGroupDraft({ name: "Grupo", description: "   " })).toEqual({
            name: "Grupo",
            description: null,
        });
    });

    it("requires a bounded non-empty name", () => {
        expect(validateRobotGroupDraft({ name: "Grupo A" })).toEqual({});
        expect(validateRobotGroupDraft({ name: "  " })).toHaveProperty("name");
        expect(validateRobotGroupDraft({ name: "A".repeat(ROBOT_GROUP_NAME_LIMIT + 1) })).toHaveProperty(
            "name"
        );
    });

    it("builds the current group lookup for every robot", () => {
        const memberships = buildRobotMemberships([
            { id: 1, name: "A", robots: [{ id: 7 }, { id: 8 }] },
            { id: 2, name: "B", robots: [{ id: 9 }] },
        ]);

        expect(memberships.get(8)).toEqual({ id: 1, name: "A" });
        expect(memberships.get(9)).toEqual({ id: 2, name: "B" });
    });

    it("counts only active inventory robots even if a disabled member remains in a group", () => {
        const memberships = buildRobotMemberships([{ id: 1, name: "A", robots: [{ id: 7 }, { id: 8 }] }]);

        expect(countAssignedActiveRobots(memberships, [{ id: 7 }])).toBe(1);
        expect(countAssignedActiveRobots(memberships, [])).toBe(0);
    });

    it("translates backend enum names and keeps a readable fallback", () => {
        expect(robotStatusLabel({ statusDescription: "En reposo", status: 1 })).toBe("En reposo");
        expect(robotStatusLabel({ statusDescription: "Idle", status: 0 })).toBe("Disponible");
        expect(robotStatusLabel({ statusDescription: "Working", status: 1 })).toBe("Ocupado");
        expect(robotStatusLabel({ statusDescription: "Disabled", status: 2 })).toBe("Deshabilitado");
        expect(robotStatusLabel({ status: 1 })).toBe("Ocupado");
        expect(robotStatusLabel({ status: 99 })).toBe("Sin estado");
    });
});
