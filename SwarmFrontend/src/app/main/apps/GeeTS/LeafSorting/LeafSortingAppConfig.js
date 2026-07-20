import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const RobotRegistryApp = lazy(() => import("./RobotRegistryApp"));

const LeafSortingAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/leafSorting",
            element: <RobotRegistryApp />,
        },
    ],
};

export default LeafSortingAppConfig;
