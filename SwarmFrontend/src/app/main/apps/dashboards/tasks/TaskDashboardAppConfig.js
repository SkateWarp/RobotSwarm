import { Navigate } from "react-router-dom";
import authRoles from "../../../../auth/authRoles";

const TaskDashboardAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    auth: authRoles.user,
    routes: [
        {
            path: "apps/dashboard/tasks",
            element: <Navigate replace to="/apps/GTS/taskLogs" />,
        },
    ],
};

export default TaskDashboardAppConfig;
