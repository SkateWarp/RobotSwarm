import { lazy } from "react";

const OperatorProductionApp = lazy(() => import("./OperatorProductionApp"));

const OperatorProductionAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    routes: [
        {
            path: "/apps/GTS/production/operators",
            element: <OperatorProductionApp />,
        },
    ],
};

export default OperatorProductionAppConfig;
