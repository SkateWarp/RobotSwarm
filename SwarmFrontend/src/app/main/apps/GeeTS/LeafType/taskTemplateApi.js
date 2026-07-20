import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import { normalizeTaskTemplateDraft } from "./taskTemplateModel";

const requestConfig = () => ({
    headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${jwtService.getAccessToken()}`,
    },
});

export const getTaskTemplateErrorMessage = (error) => {
    const response = error?.response?.data || error;
    if (typeof response === "string" && response.trim()) {
        return response;
    }

    const validationMessage = Object.values(response?.errors || {}).flat()[0];
    return (
        validationMessage ||
        response?.message ||
        response?.detail ||
        response?.title ||
        "No fue posible completar la operación."
    );
};

export const listTaskTemplates = async () => {
    const response = await axios.get(`${URL}/TaskTemplate`, requestConfig());
    return Array.isArray(response.data) ? response.data : [];
};

export const saveTaskTemplate = async ({ id, ...draft }) => {
    const response = await axios.put(
        `${URL}/TaskTemplate/${id}`,
        normalizeTaskTemplateDraft(draft),
        requestConfig()
    );
    return response.data;
};
