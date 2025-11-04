import authRoles from "../auth/authRoles";

const navigationGTSConfig = [
    {
        id: "dashboard",
        title: "Dashboard",
        type: "item",
        icon: "adjust",
        auth: authRoles.admin,
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
        title: "Task Logs",
        type: "item",
        icon: "history",
        auth: authRoles.admin,
        url: "/apps/GTS/taskLogs",
    },
    {
        id: "realtime",
        title: "Realtime",
        type: "item",
        icon: "add_task",
        auth: authRoles.admin,
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
