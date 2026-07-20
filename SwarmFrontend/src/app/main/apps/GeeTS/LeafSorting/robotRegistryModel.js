export const ROBOT_NAME_LIMIT = 100;

export const normalizeRobotDraft = (draft) => ({
    name: typeof draft?.name === "string" ? draft.name.trim() : "",
    description:
        typeof draft?.description === "string" && draft.description.trim() ? draft.description.trim() : null,
    notes: typeof draft?.notes === "string" && draft.notes.trim() ? draft.notes.trim() : null,
    status: Number.isInteger(Number(draft?.status)) ? Number(draft.status) : 0,
    isPublic: Boolean(draft?.isPublic),
});

export const validateRobotDraft = (draft) => {
    const normalized = normalizeRobotDraft(draft);
    const errors = {};
    if (!normalized.name) {
        errors.name = "Ingrese un nombre para el robot.";
    } else if (normalized.name.length > ROBOT_NAME_LIMIT) {
        errors.name = `Use como máximo ${ROBOT_NAME_LIMIT} caracteres.`;
    }
    return errors;
};

export const robotRegistryStatus = (robot) => {
    const labels = { 0: "Disponible", 1: "Ocupado", 2: "Deshabilitado" };
    const backendLabels = {
        idle: labels[0],
        working: labels[1],
        disabled: labels[2],
    };
    const description = robot?.statusDescription?.trim();

    return (
        backendLabels[description?.toLocaleLowerCase("en")] ||
        description ||
        labels[robot?.status] ||
        "Sin estado"
    );
};
