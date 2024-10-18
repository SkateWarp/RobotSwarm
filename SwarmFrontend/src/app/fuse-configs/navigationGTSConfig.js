import authRoles from "../auth/authRoles";

const navigationGTSConfig = [
    {
        id: "config",
        title: "Config",
        type: "group",
        auth: authRoles.admin,
        icon: "dashboard",
        children: [
            {
                id: "task",
                title: "Tareas",
                type: "item",
                icon: "add_task",
                auth: authRoles.admin,
                url: "/apps/GTS/leafTypes",
            },
            {
                id: "realtime",
                title: "Realtime",
                type: "item",
                icon: "add_task",
                auth: authRoles.admin,
                url: "/apps/GTS/realtime",
            },
            // {
            //     id: "sensors",
            //     title: "Sensores",
            //     type: "item",
            //     icon: "adjust",
            //     auth: authRoles.admin,
            //     url: "/apps/configs/task",
            // },
            {
                id: "leafSorting",
                title: "Robots",
                type: "item",
                icon: "settings",
                auth: authRoles.admin,
                url: "/apps/GTS/leafSorting",
            },
        ],
    },
];

export default navigationGTSConfig;
