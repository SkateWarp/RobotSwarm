import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const LeafSortingConfigApp = lazy(() => import("./GroupApp"));

const GroupAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/groups",
            element: <LeafSortingConfigApp />,
        },
    ],
};

export default GroupAppConfig;
