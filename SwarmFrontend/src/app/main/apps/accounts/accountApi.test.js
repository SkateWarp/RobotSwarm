import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import jwtService from "../../../services/jwtService";
import { createAdminAccount, disableAdminAccount, patchAdminAccount } from "./accountApi";

describe("accountApi", () => {
    let mock;

    beforeEach(() => {
        mock = new AxiosMockAdapter(axios);
        jest.spyOn(jwtService, "getAccessToken").mockReturnValue("admin-token");
    });

    afterEach(() => {
        mock.restore();
        jest.restoreAllMocks();
    });

    it("creates an account through the administrator endpoint and accepts an empty response", async () => {
        mock.onPost(`${URL}/Accounts/admin`).reply(200);

        await expect(
            createAdminAccount({
                firstName: "Ada",
                lastName: "Lovelace",
                email: "ada@example.test",
                password: "a-secure-password",
                role: "User",
            })
        ).resolves.toBeNull();

        const request = mock.history.post[0];
        expect(JSON.parse(request.data)).toMatchObject({
            role: 1,
            email: "ada@example.test",
        });
        expect(request.headers.Authorization).toBe("Bearer admin-token");
    });

    it("patches only the selected account using the role value expected by ASP.NET", async () => {
        mock.onPatch(`${URL}/Accounts/12`).reply(200, { id: 12 });

        await patchAdminAccount({ id: 12, firstName: "Grace", role: "Admin" });

        const requestBody = JSON.parse(mock.history.patch[0].data);
        expect(requestBody).toEqual({ firstName: "Grace", role: 0 });
        expect(requestBody.id).toBeUndefined();
    });

    it("treats a successful delete without a body as a disabled account", async () => {
        mock.onDelete(`${URL}/Accounts/21`).reply(200);

        await expect(disableAdminAccount(21)).resolves.toBe(true);
    });
});
