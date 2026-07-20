import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import { listTaskTemplates, saveTaskTemplate } from "./taskTemplateApi";

describe("taskTemplateApi", () => {
    let mock;

    beforeEach(() => {
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("admin-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
    });

    it("loads the real template collection with the administrator token", async () => {
        const templates = [{ id: 1, name: "Transport", taskType: 1 }];
        mock.onGet(`${URL}/TaskTemplate`).reply(200, templates);

        await expect(listTaskTemplates()).resolves.toEqual(templates);
        expect(mock.history.get[0].headers.Authorization).toBe("Bearer admin-token");
    });

    it("updates one template through PUT and sends only normalized editable fields", async () => {
        mock.onPut(`${URL}/TaskTemplate/12`).reply(200, {
            id: 12,
            name: "Leader path",
            taskType: 2,
        });

        await saveTaskTemplate({
            id: 12,
            name: "  Leader path  ",
            taskType: "2",
            dateCreated: "not editable",
        });

        const request = mock.history.put[0];
        expect(request.headers.Authorization).toBe("Bearer admin-token");
        expect(JSON.parse(request.data)).toEqual({
            name: "Leader path",
            taskType: 2,
        });
    });
});
