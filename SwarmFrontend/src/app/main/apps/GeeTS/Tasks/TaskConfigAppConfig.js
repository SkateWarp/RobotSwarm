import { Navigate } from "react-router-dom";
import authRoles from "../../../../auth/authRoles";

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
            element: <Navigate replace to="/apps/GTS/task-templates" />,
        },
    ],
};

export default TaskConfigAppConfig;
