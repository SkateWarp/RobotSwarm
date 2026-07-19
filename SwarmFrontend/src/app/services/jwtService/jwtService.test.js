import axios from "axios";
import AxiosMockAdapter from "axios-mock-adapter";
import { URL } from "app/constants/constants";
import { JwtService } from "./jwtService";

const tokenWithExpiry = (expiresAt) => {
    const header = Buffer.from(JSON.stringify({ alg: "none" })).toString("base64url");
    const payload = Buffer.from(JSON.stringify({ exp: expiresAt })).toString("base64url");
    return `${header}.${payload}.signature`;
};

const createStorage = () => {
    const entries = new Map();
    return {
        getItem: (key) => entries.get(key) || null,
        setItem: (key, value) => entries.set(key, String(value)),
        removeItem: (key) => entries.delete(key),
    };
};

describe("JwtService refresh", () => {
    let mock;
    let service;

    beforeEach(() => {
        global.window = {
            addEventListener: jest.fn(),
            clearTimeout,
            localStorage: createStorage(),
            navigator: {},
            setTimeout,
        };
        mock = new AxiosMockAdapter(axios);
        service = new JwtService();
    });

    afterEach(() => {
        service.clearRefreshTimer();
        if (service.interceptorId !== null) {
            axios.interceptors.response.eject(service.interceptorId);
        }
        mock.restore();
        delete global.window;
    });

    it("shares one refresh request between concurrent callers", async () => {
        const oldAccessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 60);
        const newAccessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 1200);
        service.setSession(oldAccessToken, "old-refresh", { id: 7, role: "User" });
        mock.onPost(`${URL}/Accounts/refreshToken`).reply(200, {
            id: 7,
            role: "User",
            jwtToken: newAccessToken,
            refreshToken: "new-refresh",
        });

        const [first, second] = await Promise.all([
            service.refreshAccessToken(),
            service.refreshAccessToken(),
        ]);

        expect(mock.history.post).toHaveLength(1);
        expect(first.jwtToken).toBe(newAccessToken);
        expect(second.jwtToken).toBe(newAccessToken);
        expect(service.getRefreshToken()).toBe("new-refresh");
    });

    it("refreshes and retries an API request once after a 401", async () => {
        const oldAccessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 60);
        const newAccessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 1200);
        const endpoint = `${URL}/api/sessions`;
        service.setSession(oldAccessToken, "old-refresh", { id: 7, role: "User" });
        service.setInterceptors();

        mock.onPost(`${URL}/Accounts/refreshToken`).reply(200, {
            id: 7,
            role: "User",
            jwtToken: newAccessToken,
            refreshToken: "new-refresh",
        });
        mock.onGet(endpoint)
            .replyOnce(401)
            .onGet(endpoint)
            .reply((config) => {
                return config.headers.Authorization === `Bearer ${newAccessToken}`
                    ? [200, { ready: true }]
                    : [401];
            });

        const response = await axios.get(endpoint, {
            headers: { Authorization: `Bearer ${oldAccessToken}` },
        });

        expect(response.data).toEqual({ ready: true });
        expect(mock.history.post).toHaveLength(1);
        expect(mock.history.get).toHaveLength(2);
    });

    it("returns login errors without trying to refresh an old session", async () => {
        const accessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 60);
        service.setSession(accessToken, "old-refresh", { id: 7, role: "User" });
        service.setInterceptors();
        mock.onPost(`${URL}/Accounts/authenticate`).reply(401, {
            message: "Credenciales incorrectas.",
        });

        await expect(service.signInWithEmailAndPassword("person@example.com", "wrong")).rejects.toEqual({
            message: "Credenciales incorrectas.",
        });

        expect(mock.history.post).toHaveLength(1);
        expect(service.getRefreshToken()).toBe("old-refresh");
    });

    it("does not erase a valid session after a temporary refresh network failure", async () => {
        const accessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 60);
        service.setSession(accessToken, "still-valid", { id: 7, role: "User" });
        mock.onPost(`${URL}/Accounts/refreshToken`).networkError();

        await expect(service.refreshAccessToken()).rejects.toThrow();

        expect(service.getAccessToken()).toBe(accessToken);
        expect(service.getRefreshToken()).toBe("still-valid");
    });

    it("does not restore a session when logout happens during refresh", async () => {
        const accessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 60);
        const newAccessToken = tokenWithExpiry(Math.floor(Date.now() / 1000) + 1200);
        let finishRefresh;
        service.setSession(accessToken, "old-refresh", { id: 7, role: "User" });
        mock.onPost(`${URL}/Accounts/refreshToken`).reply(
            () =>
                new Promise((resolve) => {
                    finishRefresh = resolve;
                })
        );

        const refresh = service.refreshAccessToken();
        service.logout();
        finishRefresh([
            200,
            {
                id: 7,
                role: "User",
                jwtToken: newAccessToken,
                refreshToken: "new-refresh",
            },
        ]);

        await expect(refresh).rejects.toMatchObject({ code: "REFRESH_CANCELLED" });
        expect(service.getAccessToken()).toBeNull();
        expect(service.getRefreshToken()).toBeNull();
    });
});
