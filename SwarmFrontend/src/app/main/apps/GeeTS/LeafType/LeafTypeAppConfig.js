import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const LeafTypesConfigApp = lazy(() => import("./LeafTypeConfigApp"));

const LeafTypesAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/leafTypes",
            element: <LeafTypesConfigApp />,
        },
    ],
};

export default LeafTypesAppConfig;
