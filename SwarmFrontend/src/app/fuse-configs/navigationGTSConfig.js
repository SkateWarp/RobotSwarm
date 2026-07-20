import authRoles from "../auth/authRoles";

const navigationGTSConfig = [
    {
        id: "realtime",
        title: "Control de simulación",
        type: "item",
        icon: "settings_remote",
        auth: authRoles.user,
        url: "/apps/GTS/realtime",
    },
    {
        id: "taskLogs",
        title: "Historial de tareas",
        type: "item",
        icon: "history",
        auth: authRoles.user,
        url: "/apps/GTS/taskLogs",
    },
    {
        id: "leafSorting",
        title: "Robots",
        type: "item",
        icon: "smart_toy",
        auth: authRoles.admin,
        url: "/apps/GTS/leafSorting",
    },
    {
        id: "robotGroups",
        title: "Grupos de robots",
        type: "item",
        icon: "groups",
        auth: authRoles.admin,
        url: "/apps/GTS/robot-groups",
    },
    {
        id: "taskTemplates",
        title: "Plantillas de tareas",
        type: "item",
        icon: "add_task",
        auth: authRoles.admin,
        url: "/apps/GTS/task-templates",
    },
    {
        id: "accounts",
        title: "Usuarios",
        type: "item",
        icon: "account_box",
        auth: authRoles.admin,
        url: "/apps/accounts",
    },
];

export default navigationGTSConfig;
