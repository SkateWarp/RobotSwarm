import {
    getRefreshDelay,
    getTokenExpiry,
    isTokenValid,
    refreshWasRejected,
    shouldRetryUnauthorized,
} from "./authTokenUtils";

const tokenWithExpiry = (expiresAt) => {
    const header = Buffer.from(JSON.stringify({ alg: "none" })).toString("base64url");
    const payload = Buffer.from(JSON.stringify({ exp: expiresAt })).toString("base64url");
    return `${header}.${payload}.signature`;
};

describe("authentication token helpers", () => {
    it("reads valid expiry values without throwing on malformed tokens", () => {
        const token = tokenWithExpiry(1234);
        expect(getTokenExpiry(token)).toBe(1234000);
        expect(getTokenExpiry("not-a-jwt")).toBeNull();
        expect(getTokenExpiry(null)).toBeNull();
    });

    it("validates tokens against the supplied clock", () => {
        const token = tokenWithExpiry(120);
        expect(isTokenValid(token, 119000)).toBe(true);
        expect(isTokenValid(token, 120000)).toBe(false);
    });

    it("schedules refresh before expiry and never returns a negative delay", () => {
        const token = tokenWithExpiry(200);
        expect(getRefreshDelay(token, 30000, 100000)).toBe(70000);
        expect(getRefreshDelay(token, 30000, 190000)).toBe(0);
        expect(getRefreshDelay("bad-token", 30000, 100000)).toBeNull();
    });

    it("retries one API 401 but excludes refresh, network, and already retried requests", () => {
        const apiUrl = "https://robot.zerav.la";
        const refreshUrl = `${apiUrl}/Accounts/refreshToken`;
        const error = {
            response: { status: 401 },
            config: { url: `${apiUrl}/api/sessions` },
        };

        expect(shouldRetryUnauthorized(error, apiUrl, refreshUrl)).toBe(true);
        expect(
            shouldRetryUnauthorized(
                { ...error, config: { ...error.config, __isRetryRequest: true } },
                apiUrl,
                refreshUrl
            )
        ).toBe(false);
        expect(shouldRetryUnauthorized({ ...error, config: { url: refreshUrl } }, apiUrl, refreshUrl)).toBe(
            false
        );
        expect(
            shouldRetryUnauthorized(
                { ...error, config: { url: "https://example.com/private" } },
                apiUrl,
                refreshUrl
            )
        ).toBe(false);
        expect(shouldRetryUnauthorized({ config: error.config }, apiUrl, refreshUrl)).toBe(false);
    });

    it("distinguishes rejected refreshes from temporary network errors", () => {
        expect(refreshWasRejected({ response: { status: 400 } })).toBe(true);
        expect(refreshWasRejected({ code: "INVALID_REFRESH_RESPONSE" })).toBe(true);
        expect(refreshWasRejected(new Error("network unavailable"))).toBe(false);
        expect(refreshWasRejected({ response: { status: 500 } })).toBe(false);
    });
});
