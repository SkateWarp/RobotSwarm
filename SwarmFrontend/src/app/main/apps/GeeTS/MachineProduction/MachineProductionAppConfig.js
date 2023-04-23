import { lazy } from "react";

const MachineProductionApp = lazy(() => import("./MachineProductionApp"));
const MachineProductionDetailsPage = lazy(() => import("./MachineProductionDetailsPage"));

const MachineProductionAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    routes: [
        {
            path: "/apps/GTS/production/machines",
            element: <MachineProductionApp />,
        },
        {
            path: "/apps/GTS/production/machines/details/:date/:machineId",
            element: <MachineProductionDetailsPage />,
        },
    ],
};

export default MachineProductionAppConfig;
