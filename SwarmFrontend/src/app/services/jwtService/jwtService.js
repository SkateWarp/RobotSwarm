import FuseUtils from "@fuse/utils/FuseUtils";
import axios from "axios";
import { URL } from "app/constants/constants";
import { getRefreshDelay, isTokenValid, refreshWasRejected, shouldRetryUnauthorized } from "./authTokenUtils";
/* eslint-disable camelcase */

const ACCESS_TOKEN_KEY = "jwt_access_token";
const REFRESH_TOKEN_KEY = "jwt_refresh_token";
const USER_KEY = "jwt_user";
const REFRESH_URL = `${URL}/Accounts/refreshToken`;
const REFRESH_LOCK = "robotswarm-jwt-refresh";
const REFRESH_EARLY_BY = 90 * 1000;
const REFRESH_RETRY_DELAY = 15 * 1000;

export class JwtService extends FuseUtils.EventEmitter {
    refreshPromise = null;

    refreshTimer = null;

    interceptorId = null;

    storageListenerIsSet = false;

    waitingForAuthentication = false;

    init() {
        this.setInterceptors();
        this.setStorageListener();
        this.handleAuthentication();
    }

    setInterceptors = () => {
        if (this.interceptorId !== null) {
            return;
        }

        this.interceptorId = axios.interceptors.response.use(
            (response) => {
                return response;
            },
            async (error) => {
                if (!shouldRetryUnauthorized(error, URL, REFRESH_URL)) {
                    if (
                        error?.response?.status === 401 &&
                        error?.config?.__isRetryRequest &&
                        error.config.url?.startsWith(URL)
                    ) {
                        this.expireSession("Su sesión ha expirado.");
                    }
                    return Promise.reject(error);
                }

                const refreshToken = this.getRefreshToken();
                try {
                    await this.refreshAccessToken();
                    const accessToken = this.getAccessToken();
                    if (!accessToken) {
                        return Promise.reject(error);
                    }

                    error.config.__isRetryRequest = true;
                    error.config.headers = {
                        ...error.config.headers,
                        Authorization: `Bearer ${accessToken}`,
                    };
                    return axios(error.config);
                } catch (refreshError) {
                    this.handleRefreshFailure(refreshError, refreshToken);
                    return Promise.reject(refreshError);
                }
            }
        );
    };

    setStorageListener = () => {
        if (this.storageListenerIsSet || typeof window === "undefined") {
            return;
        }

        window.addEventListener("storage", this.handleStorageChange);
        this.storageListenerIsSet = true;
    };

    handleStorageChange = (event) => {
        if (event.key === ACCESS_TOKEN_KEY && event.newValue) {
            this.scheduleRefresh(event.newValue);
        }

        if (event.key === ACCESS_TOKEN_KEY && !event.newValue) {
            this.clearRefreshTimer();
            this.emit("onAutoLogout", "La sesión se cerró en otra pestaña.");
        }
    };

    handleAuthentication = () => {
        const access_token = this.getAccessToken();
        const refresh_token = this.getRefreshToken();
        if (!access_token && !refresh_token) {
            this.waitingForAuthentication = false;
            this.emit("onNoAccessToken");
            return;
        }

        if (!refresh_token) {
            this.expireSession("Su sesión ha expirado.");
            return;
        }

        if (this.isAuthTokenValid(access_token)) {
            this.setSession(access_token, refresh_token);
            const user = this.getStoredUser();
            if (user) {
                this.waitingForAuthentication = false;
                this.emit("onAutoLogin", user);
                return;
            }
        }

        this.waitingForAuthentication = true;
        this.refreshAccessToken()
            .then((user) => {
                this.waitingForAuthentication = false;
                this.emit("onAutoLogin", user);
            })
            .catch((error) => {
                if (refreshWasRejected(error)) {
                    this.waitingForAuthentication = false;
                    this.expireSession("Su sesión ha expirado.");
                    return;
                }

                // A temporary network problem should not destroy a refresh token that may still be valid.
                this.scheduleRefreshRetry();
                this.emit("onNoAccessToken");
            });
    };

    createUser = (data) => {
        return new Promise((resolve, reject) => {
            axios.post(`${URL}/api/Accounts/register`, data).then((response) => {
                if (response) {
                    this.setSession(response.jwtToken, response.refreshToken);
                    resolve(response);
                } else {
                    reject(response);
                }
            });
        });
    };

    createOperator = (data) => {
        return new Promise((resolve, reject) => {
            axios
                .post(`${URL}/api/Accounts/operator`, data, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${this.getAccessToken()}`,
                    },
                })
                .then((response) => {
                    if (response.data) {
                        resolve(response.data);
                    } else {
                        reject(response.data);
                    }
                })
                .catch((error) => {
                    reject(error.response.data);
                });
        });
    };

    createAccount = (data) => {
        return new Promise((resolve, reject) => {
            axios
                .post(`${URL}/api/Accounts`, data, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${this.getAccessToken()}`,
                    },
                })
                .then((response) => {
                    if (response.data) {
                        resolve(response.data);
                    } else {
                        reject(response.data);
                    }
                })
                .catch((error) => {
                    reject(error.response.data);
                });
        });
    };

    deleteAccount = (data) => {
        return new Promise((resolve, reject) => {
            axios
                .delete(`${URL}/api/Accounts/${data}`, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${this.getAccessToken()}`,
                    },
                })
                .then((response) => {
                    if (response.data) {
                        resolve(response.data);
                    } else {
                        reject(response.data);
                    }
                })
                .catch((error) => {
                    reject(error.response.data);
                });
        });
    };

    signInWithEmailAndPassword = (email, password) => {
        return new Promise((resolve, reject) => {
            axios
                .post(
                    `${URL}/Accounts/authenticate`,
                    {
                        email,
                        password,
                    },
                    {
                        // Login errors belong to this request; they must not start
                        // the refresh flow for a previous browser session.
                        __skipAuthRefresh: true,
                    }
                )
                .then((response) => {
                    if (response.data && response.data.jwtToken) {
                        this.setSession(response.data.jwtToken, response.data.refreshToken, response.data);
                        resolve(response.data);
                    } else {
                        reject(response.data);
                    }
                })
                .catch((error) => {
                    reject(error.response?.data || error);
                });
        });
    };

    signInWithToken = () => {
        return this.refreshAccessToken();
    };

    refreshAccessToken = () => {
        if (this.refreshPromise) {
            return this.refreshPromise;
        }

        const expectedRefreshToken = this.getRefreshToken();
        if (!expectedRefreshToken) {
            return Promise.reject(this.createAuthError("INVALID_REFRESH_TOKEN"));
        }

        const refresh = async () => {
            const currentRefreshToken = this.getRefreshToken();

            // Another tab may have rotated the token while this one waited for the lock.
            if (currentRefreshToken && currentRefreshToken !== expectedRefreshToken) {
                return this.getCurrentSession();
            }

            try {
                const response = await axios.post(
                    REFRESH_URL,
                    { refreshToken: currentRefreshToken },
                    { __skipAuthRefresh: true }
                );
                const session = response.data;
                if (
                    !session ||
                    typeof session.jwtToken !== "string" ||
                    typeof session.refreshToken !== "string" ||
                    !this.isAuthTokenValid(session.jwtToken)
                ) {
                    throw this.createAuthError("INVALID_REFRESH_RESPONSE");
                }

                // Do not restore an old session when the user logged out or changed accounts mid-request.
                if (this.getRefreshToken() !== currentRefreshToken) {
                    if (this.getRefreshToken() && this.isAuthTokenValid(this.getAccessToken())) {
                        return this.getCurrentSession();
                    }
                    throw this.createAuthError("REFRESH_CANCELLED");
                }

                this.setSession(session.jwtToken, session.refreshToken, session);
                return session;
            } catch (error) {
                // Without Web Locks two tabs can still race. The tab that lost should use the winner's tokens.
                if (
                    this.getRefreshToken() &&
                    this.getRefreshToken() !== currentRefreshToken &&
                    this.isAuthTokenValid(this.getAccessToken())
                ) {
                    return this.getCurrentSession();
                }
                throw error;
            }
        };

        const locks = typeof window !== "undefined" ? window.navigator?.locks : null;
        const request = locks?.request ? locks.request(REFRESH_LOCK, refresh) : refresh();

        this.refreshPromise = Promise.resolve(request).finally(() => {
            this.refreshPromise = null;
        });
        return this.refreshPromise;
    };

    getCurrentSession = () => {
        const jwtToken = this.getAccessToken();
        const refreshToken = this.getRefreshToken();
        if (!jwtToken || !refreshToken || !this.isAuthTokenValid(jwtToken)) {
            throw this.createAuthError("INVALID_REFRESH_TOKEN");
        }

        return {
            ...(this.getStoredUser() || {}),
            jwtToken,
            refreshToken,
        };
    };

    createAuthError = (code) => {
        const error = new Error("No se ha podido renovar la sesión.");
        error.code = code;
        return error;
    };

    handleRefreshFailure = (error, attemptedRefreshToken) => {
        if (
            refreshWasRejected(error) &&
            (!attemptedRefreshToken || this.getRefreshToken() === attemptedRefreshToken)
        ) {
            this.expireSession("Su sesión ha expirado.");
            return;
        }

        this.scheduleRefreshRetry();
    };

    updateUserData = (user) => {
        return axios.put(
            `${URL}/api/Accounts/`,
            {
                user,
            },
            {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${this.getAccessToken()}`,
                },
            }
        );
    };

    setSession = (access_token, refresh_token, user = null) => {
        if (access_token) {
            window.localStorage.setItem(ACCESS_TOKEN_KEY, access_token);
            if (refresh_token) {
                window.localStorage.setItem(REFRESH_TOKEN_KEY, refresh_token);
            } else {
                window.localStorage.removeItem(REFRESH_TOKEN_KEY);
            }
            if (user) {
                const { jwtToken, refreshToken, ...profile } = user;
                window.localStorage.setItem(USER_KEY, JSON.stringify(profile));
            }
            this.scheduleRefresh(access_token);
        } else {
            this.clearRefreshTimer();
            window.localStorage.removeItem(ACCESS_TOKEN_KEY);
            window.localStorage.removeItem(REFRESH_TOKEN_KEY);
            window.localStorage.removeItem(USER_KEY);
            delete axios.defaults.headers.common.Authorization;
        }
    };

    scheduleRefresh = (access_token) => {
        this.clearRefreshTimer();
        const delay = getRefreshDelay(access_token, REFRESH_EARLY_BY);
        if (delay === null) {
            return;
        }

        this.refreshTimer = window.setTimeout(() => {
            const refreshToken = this.getRefreshToken();
            this.refreshAccessToken().catch((error) => {
                this.handleRefreshFailure(error, refreshToken);
            });
        }, delay);
    };

    scheduleRefreshRetry = () => {
        this.clearRefreshTimer();
        if (!this.getRefreshToken()) {
            return;
        }
        this.refreshTimer = window.setTimeout(() => {
            const refreshToken = this.getRefreshToken();
            this.refreshAccessToken()
                .then((user) => {
                    if (this.waitingForAuthentication) {
                        this.waitingForAuthentication = false;
                        this.emit("onAutoLogin", user);
                    }
                })
                .catch((error) => {
                    this.handleRefreshFailure(error, refreshToken);
                });
        }, REFRESH_RETRY_DELAY);
    };

    clearRefreshTimer = () => {
        if (this.refreshTimer !== null) {
            window.clearTimeout(this.refreshTimer);
            this.refreshTimer = null;
        }
    };

    expireSession = (message) => {
        this.waitingForAuthentication = false;
        this.setSession(null, null);
        this.emit("onAutoLogout", message);
    };

    logout = () => {
        this.waitingForAuthentication = false;
        this.setSession(null, null);
        // window.location.replace('/login');
    };

    isAuthTokenValid = (access_token) => {
        const isValid = isTokenValid(access_token);
        if (!isValid && access_token) {
            console.warn("access token expired");
        }
        return isValid;
    };

    getAccessToken = () => {
        return window.localStorage.getItem(ACCESS_TOKEN_KEY);
    };

    getRefreshToken = () => {
        return window.localStorage.getItem(REFRESH_TOKEN_KEY);
    };

    getStoredUser = () => {
        try {
            const user = window.localStorage.getItem(USER_KEY);
            const profile = user ? JSON.parse(user) : null;
            return profile && typeof profile === "object" && !Array.isArray(profile) ? profile : null;
        } catch (error) {
            window.localStorage.removeItem(USER_KEY);
            return null;
        }
    };

    verifyEmail = (token) => {
        return axios
            .post(`${URL}/api/Accounts/verify-email`, { token })
            .then((res) => Promise.resolve(res))
            .catch((err) => Promise.reject(err));
    };

    createResetPassword = (data) => {
        return new Promise((resolve, reject) => {
            fetch(`${URL}/api/Accounts/reset-password`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(data),
            })
                .then((response) => response.json())
                .then((responseData) => {
                    if (responseData) {
                        resolve(responseData);
                    } else {
                        reject(responseData.error);
                    }
                });
        });
    };

    createForgotPasswordLink = (data) => {
        return new Promise((resolve, reject) => {
            fetch(`${URL}/api/Accounts/forgot-password`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(data),
            })
                .then((response) => response.json())
                .then((responseData) => {
                    if (responseData) {
                        resolve(responseData);
                    } else {
                        reject(responseData.error);
                    }
                });
        });
    };
}

const instance = new JwtService();

export default instance;
