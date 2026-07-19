import jwtDecode from "jwt-decode";

export const getTokenExpiry = (token) => {
    if (!token) {
        return null;
    }

    try {
        const expiresAt = Number(jwtDecode(token).exp) * 1000;
        return Number.isFinite(expiresAt) && expiresAt > 0 ? expiresAt : null;
    } catch (error) {
        return null;
    }
};

export const isTokenValid = (token, now = Date.now()) => {
    const expiresAt = getTokenExpiry(token);
    return expiresAt !== null && expiresAt > now;
};

export const getRefreshDelay = (token, refreshEarlyBy, now = Date.now()) => {
    const expiresAt = getTokenExpiry(token);
    if (expiresAt === null) {
        return null;
    }

    return Math.max(0, expiresAt - now - refreshEarlyBy);
};

const belongsToApi = (requestUrl, apiUrl) => {
    if (!requestUrl) {
        return false;
    }

    try {
        return new URL(requestUrl, apiUrl).origin === new URL(apiUrl).origin;
    } catch (error) {
        return false;
    }
};

export const shouldRetryUnauthorized = (error, apiUrl, refreshUrl) => {
    const { config, response } = error || {};
    if (response?.status !== 401 || !config || config.__isRetryRequest || config.__skipAuthRefresh) {
        return false;
    }

    if (!belongsToApi(config.url, apiUrl)) {
        return false;
    }

    return config.url?.replace(/\/+$/, "") !== refreshUrl.replace(/\/+$/, "");
};

export const refreshWasRejected = (error) => {
    if (error?.code === "INVALID_REFRESH_TOKEN" || error?.code === "INVALID_REFRESH_RESPONSE") {
        return true;
    }

    return [400, 401, 403].includes(error?.response?.status);
};
