export const TASK_TEMPLATE_NAME_LIMIT = 100;

export const TASK_TEMPLATE_TYPES = [
    { value: 1, label: "Transporte" },
    { value: 2, label: "Seguir al líder" },
    { value: 3, label: "Formación" },
];

const typeNames = {
    Transport: 1,
    FollowLeader: 2,
    Formation: 3,
};

export const normalizeTaskTemplateType = (value) => {
    if (typeof value === "string" && Object.prototype.hasOwnProperty.call(typeNames, value)) {
        return typeNames[value];
    }

    const parsed = Number(value);
    return Number.isInteger(parsed) ? parsed : null;
};

export const taskTemplateTypeLabel = (template) => {
    if (template?.taskTypeDescription) {
        return template.taskTypeDescription;
    }

    const type = normalizeTaskTemplateType(template?.taskType);
    return TASK_TEMPLATE_TYPES.find((option) => option.value === type)?.label || "Tipo desconocido";
};

export const normalizeTaskTemplateDraft = (draft) => ({
    name: typeof draft?.name === "string" ? draft.name.trim() : "",
    taskType: normalizeTaskTemplateType(draft?.taskType),
});

export const validateTaskTemplateDraft = (draft) => {
    const normalized = normalizeTaskTemplateDraft(draft);
    const errors = {};

    if (!normalized.name) {
        errors.name = "Ingrese un nombre.";
    } else if (normalized.name.length > TASK_TEMPLATE_NAME_LIMIT) {
        errors.name = `Use como máximo ${TASK_TEMPLATE_NAME_LIMIT} caracteres.`;
    }

    if (!TASK_TEMPLATE_TYPES.some((option) => option.value === normalized.taskType)) {
        errors.taskType = "Seleccione un tipo válido.";
    }

    return errors;
};
