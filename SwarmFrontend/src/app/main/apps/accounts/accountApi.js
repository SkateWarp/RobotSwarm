import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../services/jwtService";

const requestConfig = () => ({
    headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${jwtService.getAccessToken()}`,
    },
});

const roleValues = {
    Admin: 0,
    User: 1,
};

const serializeRole = (role) => {
    if (typeof role === "string" && Object.prototype.hasOwnProperty.call(roleValues, role)) {
        return roleValues[role];
    }
    return role;
};

export const createAdminAccount = async (account) => {
    const response = await axios.post(
        `${URL}/Accounts/admin`,
        { ...account, role: serializeRole(account.role) },
        requestConfig()
    );
    return response.data ?? null;
};

export const patchAdminAccount = async ({ id, ...changes }) => {
    const payload = changes.role === undefined ? changes : { ...changes, role: serializeRole(changes.role) };
    const response = await axios.patch(`${URL}/Accounts/${id}`, payload, requestConfig());
    return response.data ?? null;
};

export const disableAdminAccount = async (id) => {
    await axios.delete(`${URL}/Accounts/${id}`, requestConfig());
    return true;
};

export const reactivateAdminAccount = async (id) => {
    const response = await axios.put(`${URL}/Accounts/${id}/reactivate`, {}, requestConfig());
    return response.data ?? null;
};

export const getAccountErrorMessage = (error) => {
    const response = error?.response?.data || error;
    if (typeof response === "string") {
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
