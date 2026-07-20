import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const TaskLogApp = lazy(() => import("./TaskRunHistoryApp"));

const TaskLogAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.user,

    routes: [
        {
            path: "/apps/GTS/taskLogs",
            element: <TaskLogApp />,
        },
    ],
};

export default TaskLogAppConfig;
