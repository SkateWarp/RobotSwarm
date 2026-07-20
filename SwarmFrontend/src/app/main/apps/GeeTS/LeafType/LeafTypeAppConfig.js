import { lazy } from "react";
import { Navigate } from "react-router-dom";
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
            path: "/apps/GTS/task-templates",
            element: <LeafTypesConfigApp />,
        },
        {
            path: "/apps/GTS/leafTypes",
            element: <Navigate replace to="/apps/GTS/task-templates" />,
        },
    ],
};

export default LeafTypesAppConfig;
