import { lazy } from "react";

const TobaccoProductionApp = lazy(() => import("./TobaccoProductionApp"));
const TobaccoProductionDetailsPage = lazy(() => import("./details/TobaccoProductionDetailsPage"));
const PredictionsDetailsPage = lazy(() => import("./predictions/PredictionsDetailsPage"));

const TobaccoProductionAppConfig = {
    settings: {
        layout: {
            config: {},
        },
    },
    routes: [
        {
            path: "/apps/GTS/production/leafs",
            element: <TobaccoProductionApp />,
        },
        {
            path: "/apps/GTS/production/leafs/details/:date/:shiftId",
            element: <TobaccoProductionDetailsPage />,
        },
        {
            path: "/apps/GTS/predictions/details/:date/:operatorId/:leafName",
            element: <PredictionsDetailsPage />,
        },
    ],
};

export default TobaccoProductionAppConfig;
