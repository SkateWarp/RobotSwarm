import { Navigate } from "react-router-dom";
import authRoles from "../../../../auth/authRoles";

const RobotDetailAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    auth: authRoles.user,

    routes: [
        {
            path: "/apps/GTS/robot/:robotId",
            element: <Navigate replace to="/apps/GTS/realtime" />,
        },
    ],
};

export default RobotDetailAppConfig;
