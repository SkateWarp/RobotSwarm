import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const LeafSortingConfigApp = lazy(() => import("./LeafSortingConfigApp"));

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
            element: <LeafSortingConfigApp />,
        },
    ],
};

export default LeafSortingAppConfig;
