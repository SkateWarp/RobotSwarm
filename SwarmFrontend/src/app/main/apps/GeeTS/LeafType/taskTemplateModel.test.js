import {
    TASK_TEMPLATE_NAME_LIMIT,
    normalizeTaskTemplateDraft,
    taskTemplateTypeLabel,
    validateTaskTemplateDraft,
} from "./taskTemplateModel";

describe("taskTemplateModel", () => {
    it("normalizes the editable fields without copying response-only data", () => {
        expect(
            normalizeTaskTemplateDraft({
                id: 7,
                name: "  Move together  ",
                taskType: "2",
                taskTypeDescription: "response value",
                dateCreated: "2026-07-19",
            })
        ).toEqual({ name: "Move together", taskType: 2 });
    });

    it("accepts only a non-empty name and a supported task type", () => {
        expect(validateTaskTemplateDraft({ name: "Formation", taskType: 3 })).toEqual({});
        expect(validateTaskTemplateDraft({ name: " ", taskType: 0 })).toEqual({
            name: "Ingrese un nombre.",
            taskType: "Seleccione un tipo válido.",
        });
        expect(
            validateTaskTemplateDraft({
                name: "A".repeat(TASK_TEMPLATE_NAME_LIMIT + 1),
                taskType: 99,
            })
        ).toMatchObject({ name: expect.any(String), taskType: expect.any(String) });
    });

    it("uses the server description and has a stable fallback for numeric types", () => {
        expect(taskTemplateTypeLabel({ taskType: 1, taskTypeDescription: "Carga" })).toBe("Carga");
        expect(taskTemplateTypeLabel({ taskType: "FollowLeader" })).toBe("Seguir al líder");
        expect(taskTemplateTypeLabel({ taskType: 99 })).toBe("Tipo desconocido");
    });
});
