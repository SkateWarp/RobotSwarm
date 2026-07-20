import { lazy } from "react";
import authRoles from "../../../../auth/authRoles";

const RobotGroupsApp = lazy(() => import("./RobotGroupsApp"));

const RobotGroupsAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    auth: authRoles.admin,
    routes: [
        {
            path: "/apps/GTS/robot-groups",
            element: <RobotGroupsApp />,
        },
    ],
};

export default RobotGroupsAppConfig;
