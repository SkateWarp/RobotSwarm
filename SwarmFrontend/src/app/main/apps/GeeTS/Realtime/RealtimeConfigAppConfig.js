import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const TaskConfigApp = lazy(() => import("./RealtimeConfigApp"));

const RealtimeConfigAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/realtime",
            element: <TaskConfigApp />,
        },
    ],
};

export default RealtimeConfigAppConfig;
