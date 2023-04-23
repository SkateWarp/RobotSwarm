import FuseUtils from "@fuse/utils/FuseUtils";
import axios from "axios";
import jwtDecode from "jwt-decode";
import { URL } from "app/constants/constants";
/* eslint-disable camelcase */

class JwtService extends FuseUtils.EventEmitter {
    init() {
        this.setInterceptors();
        this.handleAuthentication();
    }

    setInterceptors = () => {
        axios.interceptors.response.use(
            (response) => {
                return response;
            },
            (err) => {
                return new Promise(() => {
                    if (err.response.status === 401 && err.config && !err.config.__isRetryRequest) {
                        // if you ever get an unauthorized response, logout the user
                        this.emit("onAutoLogout", "Invalid access_token");
                        this.setSession(null);
                    }
                    throw err;
                });
            }
        );
    };

    handleAuthentication = () => {
        const access_token = this.getAccessToken();
        const refresh_token = this.getRefreshToken();
        if (!access_token) {
            this.emit("onNoAccessToken");

            return;
        }

        if (this.isAuthTokenValid(access_token)) {
            this.setSession(access_token, refresh_token);
            this.emit("onAutoLogin", true);
        } else {
            this.setSession(null, null);
            this.emit("onAutoLogout", "Su sesión ha expirado.");
        }
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
                .post(`${URL}/Accounts/authenticate`, {
                    email,
                    password,
                })
                .then((response) => {
                    if (response.data && response.data.jwtToken) {
                        this.setSession(response.data.jwtToken, response.data.refreshToken);
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

    signInWithToken = () => {
        return new Promise((resolve, reject) => {
            axios
                .post(`${URL}/api/Accounts/refresh-token-flutter`, {
                    refreshToken: this.getRefreshToken(),
                })
                .then((response) => {
                    if (response.data && response.data.jwtToken) {
                        this.setSession(response.data.jwtToken, response.data.refreshToken);
                        resolve(response.data);
                    } else {
                        this.logout();
                        reject(new Error("No se ha podido iniciar sesión automáticamente."));
                    }
                })
                .catch(() => {
                    this.logout();
                    reject(new Error("No se ha podido iniciar sesión automáticamente."));
                });
        });
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

    setSession = (access_token, refresh_token) => {
        if (access_token) {
            localStorage.setItem("jwt_access_token", access_token);
            localStorage.setItem("jwt_refresh_token", refresh_token);
            this.getRefreshToken();
            // axios.defaults.headers.common.Authorization = `Bearer ${access_token}`;
        } else {
            localStorage.removeItem("jwt_access_token");
            localStorage.removeItem("jwt_refresh_token");
            delete axios.defaults.headers.common.Authorization;
        }
    };

    logout = () => {
        this.setSession(null, null);
        // window.location.replace('/login');
    };

    isAuthTokenValid = (access_token) => {
        if (!access_token) {
            return false;
        }
        const decoded = jwtDecode(access_token);
        const currentTime = Date.now() / 1000;
        if (decoded.exp < currentTime) {
            console.warn("access token expired");
            return false;
        }

        return true;
    };

    getAccessToken = () => {
        return window.localStorage.getItem("jwt_access_token");
    };

    getRefreshToken = () => {
        return window.localStorage.getItem("jwt_refresh_token");
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
                .then((data) => {
                    if (data) {
                        resolve(data);
                    } else {
                        reject(data.error);
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
                .then((data) => {
                    if (data) {
                        resolve(data);
                    } else {
                        reject(data.error);
                    }
                });
        });
    };
}

const instance = new JwtService();

export default instance;
