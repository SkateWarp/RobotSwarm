import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const TaskConfigApp = lazy(() => import("./TaskConfigApp"));

const TaskConfigAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },

    auth: authRoles.admin,

    routes: [
        {
            path: "/apps/configs/task",
            element: <TaskConfigApp />,
        },
    ],
};

export default TaskConfigAppConfig;
