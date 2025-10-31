import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const RobotDetailApp = lazy(() => import("./RobotDetailApp"));

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
            element: <RobotDetailApp />,
        },
    ],
};

export default RobotDetailAppConfig;
