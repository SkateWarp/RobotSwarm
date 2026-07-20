import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import {
    addRobotToGroup,
    createRobotGroup,
    deleteRobotGroup,
    listGroupRobots,
    listRobotGroups,
    removeRobotFromGroup,
    updateRobotGroup,
} from "./robotGroupApi";

describe("robotGroupApi", () => {
    let mock;

    beforeEach(() => {
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("admin-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
    });

    it("loads groups and the complete administrator robot inventory", async () => {
        mock.onGet(`${URL}/RobotGroups`).reply(200, [{ id: 1, name: "A" }]);
        mock.onGet(`${URL}/RobotGroups/robots`).reply(200, [{ id: 5, name: "tb3_0" }]);

        await expect(listRobotGroups()).resolves.toHaveLength(1);
        await expect(listGroupRobots()).resolves.toHaveLength(1);
        expect(
            mock.history.get.every((request) => request.headers.Authorization === "Bearer admin-token")
        ).toBe(true);
    });

    it("creates and updates normalized group fields", async () => {
        mock.onPost(`${URL}/RobotGroups`).reply(200, { id: 2 });
        mock.onPut(`${URL}/RobotGroups/2`).reply(200, { id: 2 });

        await createRobotGroup({ name: "  Equipo A  ", description: "  Norte  ", robots: [] });
        await updateRobotGroup(2, { name: " Equipo B ", description: " " });

        expect(JSON.parse(mock.history.post[0].data)).toEqual({
            name: "Equipo A",
            description: "Norte",
        });
        expect(JSON.parse(mock.history.put[0].data)).toEqual({
            name: "Equipo B",
            description: null,
        });
    });

    it("uses explicit forceTransfer only when moving a robot", async () => {
        mock.onPost(`${URL}/RobotGroups/3/robots`).reply(200, { id: 3 });

        await addRobotToGroup(3, 12, true);

        expect(JSON.parse(mock.history.post[0].data)).toEqual({ robotId: 12, forceTransfer: true });
    });

    it("removes memberships and groups through their real endpoints", async () => {
        mock.onDelete(`${URL}/RobotGroups/3/robots/12`).reply(200, { id: 3 });
        mock.onDelete(`${URL}/RobotGroups/3`).reply(200, true);

        await removeRobotFromGroup(3, 12);
        await deleteRobotGroup(3);

        expect(mock.history.delete.map((request) => request.url)).toEqual([
            `${URL}/RobotGroups/3/robots/12`,
            `${URL}/RobotGroups/3`,
        ]);
    });
});
