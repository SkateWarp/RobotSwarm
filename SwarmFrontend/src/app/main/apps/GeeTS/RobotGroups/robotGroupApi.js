import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import { normalizeRobotGroupDraft } from "./robotGroupModel";

const requestConfig = () => ({
    headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${jwtService.getAccessToken()}`,
    },
});

export const getRobotGroupErrorMessage = (error) => {
    const response = error?.response?.data || error;
    if (typeof response === "string" && response.trim()) return response;
    return (
        response?.message || response?.detail || response?.title || "No fue posible completar la operación."
    );
};

export const listRobotGroups = async () => {
    const response = await axios.get(`${URL}/RobotGroups`, requestConfig());
    return Array.isArray(response.data) ? response.data : [];
};

export const listGroupRobots = async () => {
    const response = await axios.get(`${URL}/RobotGroups/robots`, requestConfig());
    return Array.isArray(response.data) ? response.data : [];
};

export const createRobotGroup = async (draft) => {
    const response = await axios.post(`${URL}/RobotGroups`, normalizeRobotGroupDraft(draft), requestConfig());
    return response.data;
};

export const updateRobotGroup = async (id, draft) => {
    const response = await axios.put(
        `${URL}/RobotGroups/${id}`,
        normalizeRobotGroupDraft(draft),
        requestConfig()
    );
    return response.data;
};

export const deleteRobotGroup = async (id) => {
    await axios.delete(`${URL}/RobotGroups/${id}`, requestConfig());
};

export const addRobotToGroup = async (groupId, robotId, forceTransfer = false) => {
    const response = await axios.post(
        `${URL}/RobotGroups/${groupId}/robots`,
        { robotId, forceTransfer },
        requestConfig()
    );
    return response.data;
};

export const removeRobotFromGroup = async (groupId, robotId) => {
    const response = await axios.delete(`${URL}/RobotGroups/${groupId}/robots/${robotId}`, requestConfig());
    return response.data;
};
