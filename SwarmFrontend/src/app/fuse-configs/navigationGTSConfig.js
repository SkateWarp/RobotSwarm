import authRoles from "../auth/authRoles";

const navigationGTSConfig = [
    {
        id: "realtime",
        title: "Control",
        type: "item",
        icon: "settings_remote",
        auth: authRoles.user,
        url: "/apps/GTS/realtime",
    },
    {
        id: "taskLogs",
        title: "Historial",
        type: "item",
        icon: "history",
        auth: authRoles.user,
        url: "/apps/GTS/taskLogs",
    },
    {
        id: "taskTemplates",
        title: "Plantillas",
        type: "item",
        icon: "add_task",
        auth: authRoles.admin,
        url: "/apps/GTS/task-templates",
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
        title: "Grupos",
        type: "item",
        icon: "groups",
        auth: authRoles.admin,
        url: "/apps/GTS/robot-groups",
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
