import authRoles from "../auth/authRoles";
import useActualProjectName from "../shared-components/hooks/useActualProjectName";

// eslint-disable-next-line react-hooks/rules-of-hooks
const actualProjectName = useActualProjectName();

const navigationGTSConfig = [
    // {
    //     id: "dashboard",
    //     title: "Dashboard",
    //     type: "group",
    //     icon: "dashboard",
    //     auth: authRoles.user,
    //     children: [
    //         {
    //             id: "dashboardBooth",
    //             title: "Cabinas",
    //             type: "item",
    //             icon: "settings_applications",
    //             url: "/apps/GTS/dashboard/booths",
    //         },
    //         {
    //             id: "dashboardShifts",
    //             title: "Turnos",
    //             type: "item",
    //             icon: "today",
    //             url: "/apps/GTS/dashboard/shifts",
    //         },
    //     ],
    // },
    // {
    //     id: "production",
    //     title: "Producción",
    //     type: "group",
    //     icon: "dashboard",
    //     auth: authRoles.user,
    //     children: [
    //         {
    //             id: "leafProduction",
    //             title: "Producción de Hojas",
    //             type: "item",
    //             icon: "today",
    //             url: "/apps/GTS/production/leafs",
    //         },
    //         {
    //             id: "operatorProduction",
    //             title: "Producción de Operadores",
    //             type: "item",
    //             icon: "account_box",
    //             url: "/apps/GTS/production/operators",
    //         },
    //         {
    //             id: "machineProduction",
    //             title: "Producción de Máquinas",
    //             type: "item",
    //             icon: "settings_applications",
    //             url: "/apps/GTS/production/machines",
    //         },
    //     ],
    // },
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
                id: "sensors",
                title: "Sensores",
                type: "item",
                icon: "adjust",
                auth: authRoles.admin,
                url: "/apps/configs/task",
            },
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
