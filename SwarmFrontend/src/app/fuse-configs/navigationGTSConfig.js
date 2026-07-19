import authRoles from "../auth/authRoles";

const navigationGTSConfig = [
    {
        id: "dashboard",
        title: "Dashboard",
        type: "item",
        icon: "adjust",
        auth: authRoles.user,
        url: "/apps/GTS/dashboard/booths/",
    },
    {
        id: "task",
        title: "Tareas",
        type: "item",
        icon: "add_task",
        auth: authRoles.admin,
        url: "/apps/GTS/leafTypes",
    },
    {
        id: "taskLogs",
        title: "Historial de Tareas",
        type: "item",
        icon: "history",
        auth: authRoles.admin,
        url: "/apps/GTS/taskLogs",
    },
    {
        id: "realtime",
        title: "Control en Tiempo Real",
        type: "item",
        icon: "settings_remote",
        auth: authRoles.user,
        url: "/apps/GTS/realtime",
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
        id: "accounts",
        title: "Cuentas",
        type: "item",
        icon: "account_box",
        auth: authRoles.admin,
        url: "/apps/accounts",
    },
];

export default navigationGTSConfig;
