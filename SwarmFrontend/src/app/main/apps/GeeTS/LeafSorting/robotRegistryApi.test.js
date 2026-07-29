import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import {
    createRegistryRobot,
    disableRegistryRobot,
    getRobotRegistryErrorMessage,
    listRegistryRobots,
    reactivateRegistryRobot,
    updateRegistryRobot,
} from "./robotRegistryApi";

describe("robotRegistryApi", () => {
    let mock;

    beforeEach(() => {
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("admin-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
    });

    it("loads the administrator registry with authentication", async () => {
        mock.onGet(`${URL}/Robots`).reply(200, [
            { id: 1, name: "publico" },
            { id: 2, name: "privado-ajeno" },
        ]);

        await expect(listRegistryRobots()).resolves.toHaveLength(2);
        expect(mock.history.get[0].headers.Authorization).toBe("Bearer admin-token");
        expect(mock.history.get[0].params).toEqual({ includeDisabled: true });
    });

    it("sends only normalized editable fields when creating and updating", async () => {
        mock.onPost(`${URL}/Robots`).reply(200, { id: 4 });
        mock.onPut(`${URL}/Robots/4`).reply(200, { id: 4 });
        const draft = {
            id: 4,
            name: "  tb3_0  ",
            description: "  Explorador  ",
            notes: "  Revisado  ",
            status: 1,
            isPublic: true,
            accountId: 999,
            namespace: "/inyectado",
        };

        await createRegistryRobot(draft);
        await updateRegistryRobot(4, draft);

        const expected = {
            name: "tb3_0",
            description: "Explorador",
            notes: "Revisado",
            status: 1,
            isPublic: true,
        };
        expect(JSON.parse(mock.history.post[0].data)).toEqual(expected);
        expect(JSON.parse(mock.history.put[0].data)).toEqual(expected);
        expect(mock.history.put[0].headers.Authorization).toBe("Bearer admin-token");
    });

    it("disables through the real update endpoint without changing ownership", async () => {
        mock.onPut(`${URL}/Robots/9`).reply(200, { id: 9, status: 2 });

        await disableRegistryRobot({
            id: 9,
            name: "tb3_9",
            description: null,
            notes: null,
            status: 0,
            isPublic: false,
            accountId: 21,
        });

        expect(JSON.parse(mock.history.put[0].data)).toEqual({
            name: "tb3_9",
            description: null,
            notes: null,
            status: 2,
            isPublic: false,
        });
    });

    it("reactivates through the same validated update contract", async () => {
        mock.onPut(`${URL}/Robots/9`).reply(200, { id: 9, status: 0 });

        await reactivateRegistryRobot({
            id: 9,
            name: "tb3_9",
            description: "Explorador",
            notes: null,
            status: 2,
            isPublic: false,
            accountId: 21,
            namespace: "tb3_9",
        });

        expect(JSON.parse(mock.history.put[0].data)).toEqual({
            name: "tb3_9",
            description: "Explorador",
            notes: null,
            status: 0,
            isPublic: false,
        });
    });

    it("returns safe empty data and readable server errors", async () => {
        mock.onGet(`${URL}/Robots`).reply(200, { items: [] });

        await expect(listRegistryRobots()).resolves.toEqual([]);
        expect(getRobotRegistryErrorMessage({ response: { data: { detail: "Sin permiso" } } })).toBe(
            "Sin permiso"
        );
        expect(getRobotRegistryErrorMessage({})).toBe("No fue posible completar la operación.");
    });
});
