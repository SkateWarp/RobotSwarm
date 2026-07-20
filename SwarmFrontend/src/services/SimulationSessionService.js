import axios from "axios";
import { HubConnectionBuilder, LogLevel } from "@microsoft/signalr";
import { URL } from "app/constants/constants";
import jwtService from "app/services/jwtService";

const API_URL = URL.replace(/\/+$/, "");
const VIEWER_CLOSE_TERMINAL_STATES = new Set(["Completed", "Failed", "Cancelled"]);

const pause = (milliseconds) =>
    new Promise((resolve) => {
        window.setTimeout(resolve, milliseconds);
    });

const authHeaders = () => {
    const token = jwtService.getAccessToken();
    return {
        "Content-Type": "application/json",
        ...(token ? { Authorization: `Bearer ${token}` } : {}),
    };
};

const commandHeaders = () => ({
    ...authHeaders(),
    "Idempotency-Key": window.crypto?.randomUUID
        ? window.crypto.randomUUID()
        : `${Date.now()}-${Math.random().toString(16).slice(2)}`,
});

const changeTask = async (sessionId, taskId, action) => {
    const response = await axios.post(
        `${API_URL}/api/sessions/${sessionId}/tasks/${taskId}/${action}`,
        {},
        { headers: commandHeaders() }
    );
    return response.data;
};

const SimulationSessionService = {
    async getLimits() {
        const response = await axios.get(`${API_URL}/api/sessions/limits`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async list() {
        const response = await axios.get(`${API_URL}/api/sessions`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async create(robotCount) {
        const response = await axios.post(
            `${API_URL}/api/sessions`,
            { robotCount },
            { headers: authHeaders() }
        );
        return response.data;
    },

    async stop(sessionId) {
        const response = await axios.delete(`${API_URL}/api/sessions/${sessionId}`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async updateFleet(sessionId, robotCount) {
        const response = await axios.patch(
            `${API_URL}/api/sessions/${sessionId}/fleet`,
            { robotCount },
            { headers: commandHeaders() }
        );
        return response.data;
    },

    async listRobots(sessionId) {
        const response = await axios.get(`${API_URL}/api/sessions/${sessionId}/robots`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async listTasks(sessionId) {
        const response = await axios.get(`${API_URL}/api/sessions/${sessionId}/tasks`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async listTaskHistory({ offset = 0, limit = 25, type, state, outcome } = {}) {
        const response = await axios.get(`${API_URL}/api/sessions/tasks/history`, {
            headers: authHeaders(),
            params: {
                offset,
                limit,
                ...(type ? { type } : {}),
                ...(state ? { state } : {}),
                ...(outcome ? { outcome } : {}),
            },
        });
        return response.data;
    },

    async startTask(sessionId, type, parameters) {
        const response = await axios.post(
            `${API_URL}/api/sessions/${sessionId}/tasks`,
            { type, parameters },
            { headers: commandHeaders() }
        );
        return response.data;
    },

    async pauseTask(sessionId, taskId) {
        return changeTask(sessionId, taskId, "pause");
    },

    async resumeTask(sessionId, taskId) {
        return changeTask(sessionId, taskId, "resume");
    },

    async cancelTask(sessionId, taskId) {
        return changeTask(sessionId, taskId, "cancel");
    },

    async emergencyStop(sessionId) {
        const response = await axios.post(
            `${API_URL}/api/sessions/${sessionId}/emergency-stop`,
            {},
            { headers: commandHeaders() }
        );
        return response.data;
    },

    async resetEmergencyStop(sessionId) {
        const response = await axios.post(
            `${API_URL}/api/sessions/${sessionId}/reset-emergency-stop`,
            {},
            { headers: commandHeaders() }
        );
        return response.data;
    },

    async createViewerLease(sessionId, source, robotRuntimeId = null) {
        const response = await axios.post(
            `${API_URL}/api/sessions/${sessionId}/viewer-lease`,
            { source, robotRuntimeId },
            { headers: commandHeaders() }
        );
        return response.data;
    },

    async getViewerLeaseStatus(sessionId, leaseId) {
        const response = await axios.get(`${API_URL}/api/sessions/${sessionId}/viewer-lease/${leaseId}`, {
            headers: authHeaders(),
        });
        return response.data;
    },

    async closeViewerLease(sessionId, leaseId) {
        const response = await axios.delete(`${API_URL}/api/sessions/${sessionId}/viewer-lease/${leaseId}`, {
            headers: authHeaders(),
        });
        return {
            ...response.data,
            accepted: response.status === 202,
        };
    },

    async waitForViewerClose(
        sessionId,
        leaseId,
        { maxAttempts = 8, pollIntervalMs = 1250, sleep = pause } = {}
    ) {
        const attempts = Math.max(1, Math.floor(maxAttempts));
        let lastStatus = null;
        let lastError = null;

        for (let attempt = 0; attempt < attempts; attempt += 1) {
            try {
                // Each check depends on the previous result; parallel requests could return out of order.
                // eslint-disable-next-line no-await-in-loop
                const response = await axios.get(
                    `${API_URL}/api/sessions/${sessionId}/viewer-lease/${leaseId}`,
                    { headers: authHeaders() }
                );
                lastStatus = response.data;
                lastError = null;
                const state = lastStatus?.closeCommand?.state;
                if (VIEWER_CLOSE_TERMINAL_STATES.has(state)) {
                    return {
                        state,
                        command: lastStatus.closeCommand,
                        status: lastStatus,
                        timedOut: false,
                    };
                }
            } catch (requestError) {
                lastError = requestError;
            }

            if (attempt + 1 < attempts) {
                // Keep the polling rate bounded instead of filling the backend queue.
                // eslint-disable-next-line no-await-in-loop
                await sleep(pollIntervalMs);
            }
        }

        return {
            state: "TimedOut",
            command: lastStatus?.closeCommand || null,
            status: lastStatus,
            error: lastError,
            timedOut: true,
        };
    },

    createRealtimeConnection() {
        return new HubConnectionBuilder()
            .withUrl(`${API_URL}/hubs/session`, {
                accessTokenFactory: () => jwtService.getAccessToken() || "",
            })
            .withAutomaticReconnect([0, 1000, 3000, 5000, 10000])
            .configureLogging(LogLevel.Warning)
            .build();
    },
};

export default SimulationSessionService;
