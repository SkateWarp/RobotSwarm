import { Navigate } from "react-router-dom";
import FuseUtils from "@fuse/utils";
import FuseLoading from "@fuse/core/FuseLoading";
import LoginConfig from "app/main/login/LoginConfig";
import MailConfirmPageConfig from "app/main/mail-confirm/MailConfirmPageConfig";
import ResetPasswordPageConfig from "app/main/reset-password/ResetPasswordPageConfig";
import AccountsAppConfig from "app/main/apps/accounts/AccountsAppConfig";
import RealtimeConfigAppConfig from "app/main/apps/GeeTS/Realtime/RealtimeConfigAppConfig";
import ForgotPasswordPageConfig from "../main/forgot-password/ForgotPasswordPageConfig";
import TaskDashboardAppConfig from "../main/apps/dashboards/tasks/TaskDashboardAppConfig";
import settingsConfig from "./settingsConfig";
import TaskConfigAppConfig from "../main/apps/GeeTS/Tasks/TaskConfigAppConfig";

// GeeTS
import LeafTypesAppConfig from "../main/apps/GeeTS/LeafType/LeafTypeAppConfig";
import BoothDashboardAppConfigGeeTS from "../main/apps/GeeTS/Dashboard/BoothDashboardAppConfig";
import LeafSortingAppConfig from "../main/apps/GeeTS/LeafSorting/LeafSortingAppConfig";
import GroupAppConfig from "../main/apps/GeeTS/Groups/GroupAppConfig";

const routeConfigs = [
    LoginConfig,
    MailConfirmPageConfig,
    ResetPasswordPageConfig,
    AccountsAppConfig,
    ForgotPasswordPageConfig,

    // Tasks
    TaskDashboardAppConfig,
    TaskConfigAppConfig,
    RealtimeConfigAppConfig,

    // GeeTS
    LeafTypesAppConfig,
    LeafSortingAppConfig,
    BoothDashboardAppConfigGeeTS,
    GroupAppConfig
];

const getInitialDashboard = () => {
    if (settingsConfig.layout.project === "task") {
        return { path: "/", exact: true, element: <Navigate to="apps/dashboard/tasks" /> };
    }

    if (settingsConfig.layout.project === "GTS" || settingsConfig.layout.project === "GTS-swedish") {
        return { path: "/", exact: true, element: <Navigate to="/apps/GTS/leafSorting" /> };
    }

    throw Error("Proyecto mal configurado en routesConfig");
};

const routes = [
    // if you want to make whole app auth protected by default change defaultAuth for example:
    // ...FuseUtils.generateRoutesFromConfigs(routeConfigs, ['admin','staff','user']),
    // The individual route configs which has auth option won't be overridden.
    ...FuseUtils.generateRoutesFromConfigs(routeConfigs, null),

    getInitialDashboard(),
    {
        path: "loading",
        element: <FuseLoading />,
    },
    {
        path: "*",
        element: <Navigate to="pages/errors/error-404" />,
    },
];

export default routes;
