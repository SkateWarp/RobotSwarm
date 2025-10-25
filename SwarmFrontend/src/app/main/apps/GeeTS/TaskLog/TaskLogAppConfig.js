import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const TaskLogApp = lazy(() => import("./TaskLogApp"));

const TaskLogAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/GTS/taskLogs",
            element: <TaskLogApp />,
        },
    ],
};

export default TaskLogAppConfig;
