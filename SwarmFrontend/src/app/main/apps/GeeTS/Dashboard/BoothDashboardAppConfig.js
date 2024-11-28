import { lazy } from "react";
import { Navigate } from "react-router-dom";
import authRoles from "../../../../auth/authRoles";

const BoothDashboardApp = lazy(() => import("./booths/RobotDashboardApp"));
const ShiftsDashboardApp = lazy(() => import("./shifts/ShiftsDashboardApp"));
const ShiftsDashboardDetails = lazy(() => import("./shifts/ShiftsDashboardDetails"));
const BoothDashboardDetails = lazy(() => import("./booths/RobotDashboardDetails"));

const BoothDashboardAppConfigGeeTS = {
    settings: {
        layout: {
            config: {},
        },
    },
    auth: authRoles.user,

    routes: [
        {
            path: "/apps/GTS/dashboard/booths/",
            element: <BoothDashboardApp />,
        },
        {
            path: "/apps/GTS/dashboard/booths/:id",
            element: <BoothDashboardApp />,
        },
        {
            path: `/apps/GTS/dashboard/booths/:date/:machineId`,
            element: <BoothDashboardDetails />,
        },
        {
            path: "/apps/GTS/dashboard/shifts",
            element: <ShiftsDashboardApp />,
        },
        {
            path: `/apps/GTS/dashboard/shifts/:date/:shiftId`,
            element: <ShiftsDashboardDetails />,
        },
        {
            path: "/apps/GTS/dashboard",
            element: <Navigate to="/apps/GTS/dashboard/booths" />,
        },
    ],
};

export default BoothDashboardAppConfigGeeTS;
