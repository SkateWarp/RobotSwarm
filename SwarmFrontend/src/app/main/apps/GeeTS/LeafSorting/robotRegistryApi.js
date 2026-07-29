import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import { normalizeRobotDraft } from "./robotRegistryModel";

const requestConfig = () => ({
    headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${jwtService.getAccessToken()}`,
    },
});

export const getRobotRegistryErrorMessage = (error) => {
    const response = error?.response?.data || error;
    if (typeof response === "string" && response.trim()) return response;
    return (
        response?.message || response?.detail || response?.title || "No fue posible completar la operación."
    );
};

export const listRegistryRobots = async () => {
    const response = await axios.get(`${URL}/Robots`, {
        ...requestConfig(),
        params: { includeDisabled: true },
    });
    return Array.isArray(response.data) ? response.data : [];
};

export const createRegistryRobot = async (draft) => {
    const response = await axios.post(`${URL}/Robots`, normalizeRobotDraft(draft), requestConfig());
    return response.data;
};

export const updateRegistryRobot = async (id, draft) => {
    const response = await axios.put(`${URL}/Robots/${id}`, normalizeRobotDraft(draft), requestConfig());
    return response.data;
};

export const disableRegistryRobot = async (robot) => {
    const response = await axios.put(
        `${URL}/Robots/${robot.id}`,
        { ...normalizeRobotDraft(robot), status: 2 },
        requestConfig()
    );
    return response.data;
};

export const reactivateRegistryRobot = async (robot) => {
    const response = await axios.put(
        `${URL}/Robots/${robot.id}`,
        { ...normalizeRobotDraft(robot), status: 0 },
        requestConfig()
    );
    return response.data;
};
