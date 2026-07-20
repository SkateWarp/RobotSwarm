export const ROBOT_GROUP_NAME_LIMIT = 80;

export const normalizeRobotGroupDraft = (draft) => ({
    name: typeof draft?.name === "string" ? draft.name.trim() : "",
    description:
        typeof draft?.description === "string" && draft.description.trim() ? draft.description.trim() : null,
});

export const validateRobotGroupDraft = (draft) => {
    const normalized = normalizeRobotGroupDraft(draft);
    const errors = {};

    if (!normalized.name) {
        errors.name = "Ingrese un nombre para el grupo.";
    } else if (normalized.name.length > ROBOT_GROUP_NAME_LIMIT) {
        errors.name = `Use como máximo ${ROBOT_GROUP_NAME_LIMIT} caracteres.`;
    }

    return errors;
};

export const buildRobotMemberships = (groups) => {
    const memberships = new Map();
    (Array.isArray(groups) ? groups : []).forEach((group) => {
        (Array.isArray(group?.robots) ? group.robots : []).forEach((robot) => {
            memberships.set(robot.id, { id: group.id, name: group.name });
        });
    });
    return memberships;
};

export const countAssignedActiveRobots = (memberships, robots) => {
    const activeRobotIds = new Set((Array.isArray(robots) ? robots : []).map((robot) => robot.id));
    return [...(memberships instanceof Map ? memberships.keys() : [])].filter((robotId) =>
        activeRobotIds.has(robotId)
    ).length;
};

export const robotStatusLabel = (robot) => {
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
