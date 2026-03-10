import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const SwarmControlApp = lazy(() => import("./SwarmControlApp"));

const SwarmControlAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/swarm-control",
            element: <SwarmControlApp />,
        },
    ],
};

export default SwarmControlAppConfig;
