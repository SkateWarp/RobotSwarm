import { Navigate } from "react-router-dom";
import authRoles from "../auth/authRoles";
import AccountsAppConfig from "../main/apps/accounts/AccountsAppConfig";
import LeafSortingAppConfig from "../main/apps/GeeTS/LeafSorting/LeafSortingAppConfig";
import LeafTypesAppConfig from "../main/apps/GeeTS/LeafType/LeafTypeAppConfig";
import RobotDetailAppConfig from "../main/apps/GeeTS/RobotDetail/RobotDetailAppConfig";
import RobotGroupsAppConfig from "../main/apps/GeeTS/RobotGroups/RobotGroupsAppConfig";
import TaskConfigAppConfig from "../main/apps/GeeTS/Tasks/TaskConfigAppConfig";
import TaskLogAppConfig from "../main/apps/GeeTS/TaskLog/TaskLogAppConfig";
import TaskDashboardAppConfig from "../main/apps/dashboards/tasks/TaskDashboardAppConfig";
import navigationGTSConfig from "./navigationGTSConfig";

const navigationItem = (id) => navigationGTSConfig.find((item) => item.id === id);
const route = (config, path) => config.routes.find((item) => item.path === path);

describe("GTS section navigation", () => {
    it("exposes every active management section at its real route", () => {
        expect(navigationGTSConfig.map((item) => item.title)).toEqual([
            "Control",
            "Historial",
            "Plantillas",
            "Robots",
            "Grupos",
            "Usuarios",
        ]);
        expect(navigationItem("taskTemplates")).toMatchObject({
            title: "Plantillas",
            url: "/apps/GTS/task-templates",
            auth: authRoles.admin,
        });
        expect(navigationItem("taskLogs")).toMatchObject({
            title: "Historial",
            url: "/apps/GTS/taskLogs",
            auth: authRoles.user,
        });
        expect(navigationItem("leafSorting")).toMatchObject({
            title: "Robots",
            url: "/apps/GTS/leafSorting",
            auth: authRoles.admin,
        });
        expect(navigationItem("robotGroups")).toMatchObject({
            title: "Grupos",
            url: "/apps/GTS/robot-groups",
            auth: authRoles.admin,
        });
        expect(navigationItem("accounts")).toMatchObject({
            title: "Usuarios",
            url: "/apps/accounts",
            auth: authRoles.admin,
        });
    });

    it("keeps route authorization aligned with the visible navigation", () => {
        expect(LeafTypesAppConfig.auth).toEqual(authRoles.admin);
        expect(TaskLogAppConfig.auth).toEqual(authRoles.user);
        expect(LeafSortingAppConfig.auth).toEqual(authRoles.admin);
        expect(RobotGroupsAppConfig.auth).toEqual(authRoles.admin);
        expect(AccountsAppConfig.auth).toEqual(authRoles.admin);

        expect(route(LeafTypesAppConfig, "/apps/GTS/task-templates")).toBeDefined();
        expect(route(TaskLogAppConfig, "/apps/GTS/taskLogs")).toBeDefined();
        expect(route(LeafSortingAppConfig, "/apps/GTS/leafSorting")).toBeDefined();
        expect(route(RobotGroupsAppConfig, "/apps/GTS/robot-groups")).toBeDefined();
        expect(route(AccountsAppConfig, "/apps/accounts/:param")).toBeDefined();
    });

    it("replaces the short Usuarios URL so browser Back is not trapped by the redirect", () => {
        const redirect = route(AccountsAppConfig, "/apps/accounts");

        expect(redirect.element.props.to).toBe("/apps/accounts/all");
        expect(redirect.element.props.replace).toBe(true);
    });

    it("keeps legacy robot links inside the user-accessible simulation control", () => {
        const redirect = route(RobotDetailAppConfig, "/apps/GTS/robot/:robotId");

        expect(RobotDetailAppConfig.auth).toEqual(authRoles.user);
        expect(redirect.element.props.to).toBe("/apps/GTS/realtime");
        expect(redirect.element.props.replace).toBe(true);
    });

    it("redirects legacy task pages to the real task sources", () => {
        const taskConfig = route(TaskConfigAppConfig, "/apps/configs/task");
        const taskDashboard = route(TaskDashboardAppConfig, "apps/dashboard/tasks");

        expect(TaskConfigAppConfig.auth).toEqual(authRoles.admin);
        expect(taskConfig.element.type).toBe(Navigate);
        expect(taskConfig.element.props.to).toBe("/apps/GTS/task-templates");
        expect(taskConfig.element.props.replace).toBe(true);

        expect(TaskDashboardAppConfig.auth).toEqual(authRoles.user);
        expect(taskDashboard.element.type).toBe(Navigate);
        expect(taskDashboard.element.props.to).toBe("/apps/GTS/taskLogs");
        expect(taskDashboard.element.props.replace).toBe(true);
    });
});
