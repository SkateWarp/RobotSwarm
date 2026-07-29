import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../../services/jwtService";
import { registerError, registerSuccess } from "../../../../auth/store/registerSlice";
import { showMessage } from "../../../../store/fuse/messageSlice";
import {
    createAdminAccount,
    disableAdminAccount,
    getAccountErrorMessage,
    patchAdminAccount,
    reactivateAdminAccount,
} from "../accountApi";
import { ACCOUNTS_PAGE_SIZE, normalizeAccount } from "../accountsViewModel";
// import HEADERS from "../../../../constants/authorizationHeaders";

export const getMachinesForNotificationsByAccountIdAndAlertType = (id, machineId) => {
    return new Promise((resolve) => {
        axios.get(`${URL}/api/Alarm/accounts-alarms/${id}`, { params: { machineId } }).then((res) => {
            resolve(res.data);
        });
    });
};

export const getAccountPermission = (id) => {
    return new Promise((resolve) => {
        axios.get(`${URL}/api/Accounts/accounts-permission/${id}`).then((res) => {
            resolve(res.data);
        });
    });
};

export const createNotificationAccountRequest = (data) => {
    return new Promise((resolve) => {
        axios.post(`${URL}/api/Alarm/accounts-alarms`, data).then((res) => {
            resolve(res.data);
        });
    });
};

export const createPermissionAccountRequest = (data) => {
    return new Promise((resolve) => {
        axios.post(`${URL}/api/Accounts/accounts-permission`, data).then((res) => {
            resolve(res.data);
        });
    });
};

export const submitAccount = (model) => async (dispatch) => {
    return createAdminAccount(model)
        .then((user) => {
            return dispatch(registerSuccess(user));
        })
        .catch((error) => {
            dispatch(
                showMessage({
                    message: getAccountErrorMessage(error), // text or html
                    autoHideDuration: 3000, // ms
                    anchorOrigin: {
                        vertical: "top", // top bottom
                        horizontal: "center", // left center right
                    },
                    variant: "error", // success error info warning null
                })
            );
            return dispatch(registerError(error));
        });
};

export const removeAccount = (model) => async (dispatch) => {
    const accountId = typeof model === "object" ? model.idForDelete : model;
    return disableAdminAccount(accountId)
        .then(() => {
            dispatch(getAccounts());
            return dispatch(registerSuccess());
        })
        .catch((error) => {
            //
            dispatch(
                showMessage({
                    message: getAccountErrorMessage(error), // text or html
                    autoHideDuration: 6000, // ms
                    anchorOrigin: {
                        vertical: "top", // top bottom
                        horizontal: "center", // left center right
                    },
                    variant: "error", // success error info warning null
                })
            );

            return dispatch(registerError(error));
        });
};

export const getAccounts = createAsyncThunk(
    "accountsApp/accounts/getAccounts",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().accountsApp.accounts.routeParams;

        const response = await axios.get(`${URL}/Accounts`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });

        const data = await response.data;

        return { data, routeParams };
    }
);

export const getAccountsByCompany = createAsyncThunk(
    "accountsApp/accounts/getAccountsByCompany",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().accountsApp.accounts.routeParams;

        const { companyId } = routeParams;
        const response = await axios.get(`${URL}/api/Accounts/company/${companyId}`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });

        const data = await response.data;

        return { data, routeParams };
    }
);

export const updateAccountPanelTemp = createAsyncThunk(
    "accountsApp/accounts/updateAccountPanelTemp",
    async (account, { dispatch }) => {
        const response = await axios.put(`${URL}/api/Accounts/${account.id}`, account, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = response;
        return data;
    }
);

export const addAccount = createAsyncThunk(
    "accountsApp/accounts/addAccount",
    async (account, { dispatch, rejectWithValue }) => {
        try {
            const data = await createAdminAccount(account);
            await dispatch(getAccounts());
            return data;
        } catch (error) {
            return rejectWithValue(getAccountErrorMessage(error));
        }
    }
);

export const updateAccount = createAsyncThunk(
    "accountsApp/accounts/updateAccount",
    async (account, { dispatch, rejectWithValue }) => {
        try {
            const data = await patchAdminAccount(account);
            await dispatch(getAccounts());
            return data;
        } catch (error) {
            return rejectWithValue(getAccountErrorMessage(error));
        }
    }
);

export const disableAccount = createAsyncThunk(
    "accountsApp/accounts/disableAccount",
    async (accountId, { dispatch, rejectWithValue }) => {
        try {
            await disableAdminAccount(accountId);
            await dispatch(getAccounts());
            return accountId;
        } catch (error) {
            return rejectWithValue(getAccountErrorMessage(error));
        }
    }
);

export const reactivateAccount = createAsyncThunk(
    "accountsApp/accounts/reactivateAccount",
    async (accountId, { dispatch, rejectWithValue }) => {
        try {
            await reactivateAdminAccount(accountId);
            await dispatch(getAccounts());
            return accountId;
        } catch (error) {
            return rejectWithValue(getAccountErrorMessage(error));
        }
    }
);

const accountsAdapter = createEntityAdapter({});

export const { selectAll: selectAccounts, selectById: selectAccountsById } = accountsAdapter.getSelectors(
    (state) => state.accountsApp.accounts
);

const accountsSlice = createSlice({
    name: "accountsApp/accounts",
    initialState: accountsAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        params: "",
        accounts: [],
        accountDialog: {
            type: "new",
            props: {
                open: false,
            },
            data: null,
        },
        pagination: {
            pageSize: ACCOUNTS_PAGE_SIZE,
            pageNumber: 1,
            totalPages: 0,
            totalRecords: 0,
            searchFilter: "",
            sortColumn: "",
            sortDesc: false,
        },
        loading: false,
        error: null,
    }),
    reducers: {
        setAccountsSearchText: {
            reducer: (state, action) => {
                state.searchText = action.payload;
            },
            prepare: (event) => ({ payload: event.target.value || "" }),
        },
        setPaginationData: (state, action) => {
            const { pageSize, pageNumber } = action.payload;
            if (pageSize) state.routeParams.pageSize = pageSize;
            if (pageNumber) state.routeParams.pageNumber = pageNumber;
        },
        openNewAccountDialog: (state) => {
            state.accountDialog = {
                type: "new",
                props: {
                    open: true,
                },
                data: null,
            };
        },
        closeNewAccountDialog: (state) => {
            state.accountDialog = {
                type: "new",
                props: {
                    open: false,
                },
                data: null,
            };
        },
        openEditAccountDialog: (state, action) => {
            state.accountDialog = {
                type: "edit",
                props: {
                    open: true,
                },
                data: action.payload,
            };
        },
        closeEditAccountDialog: (state) => {
            state.accountDialog = {
                type: "edit",
                props: {
                    open: false,
                },
                data: null,
            };
        },
    },
    extraReducers: {
        [getAccounts.pending]: (state) => {
            state.loading = true;
            state.error = null;
        },
        [getAccounts.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;
            state.loading = false;
            state.error = null;
            // Handle simple array response (not paginated)
            if (Array.isArray(data)) {
                const normalizedAccounts = data.map(normalizeAccount);
                accountsAdapter.setAll(state, normalizedAccounts);
                state.accounts = normalizedAccounts;
                state.routeParams = routeParams;
                state.pagination = {
                    pageNumber: 1,
                    pageSize: ACCOUNTS_PAGE_SIZE,
                    totalPages: Math.max(1, Math.ceil(normalizedAccounts.length / ACCOUNTS_PAGE_SIZE)),
                    totalRecords: normalizedAccounts.length,
                    searchFilter: "",
                    sortColumn: "",
                    sortDesc: false,
                };
            } else {
                // Handle paginated response (legacy format)
                accountsAdapter.setAll(state, data.data);
                state.accounts = data;
                state.routeParams = routeParams;
                state.pagination = {
                    pageNumber: data.pageNumber,
                    pageSize: data.pageSize,
                    totalPages: data.totalPages,
                    totalRecords: data.totalRecords,
                    searchFilter: data.searchFilter,
                    sortColumn: data.sortColumn,
                    sortDesc: data.sortDesc,
                };
            }
        },
        [getAccounts.rejected]: (state, action) => {
            state.loading = false;
            state.error = getAccountErrorMessage(action.error);
        },
        [getAccountsByCompany.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;
            // Handle simple array response (not paginated)
            if (Array.isArray(data)) {
                accountsAdapter.setAll(state, data);
                state.accounts = data;
                state.routeParams = routeParams;
                state.pagination = {
                    pageNumber: 1,
                    pageSize: data.length,
                    totalPages: 1,
                    totalRecords: data.length,
                    searchFilter: "",
                    sortColumn: "",
                    sortDesc: false,
                };
            } else {
                // Handle paginated response (legacy format)
                accountsAdapter.setAll(state, data.data || data);
                state.accounts = data;
                state.routeParams = routeParams;
                state.pagination = {
                    pageNumber: data.pageNumber,
                    pageSize: data.pageSize,
                    totalPages: data.totalPages,
                    totalRecords: data.totalRecords,
                    searchFilter: data.searchFilter,
                    sortColumn: data.sortColumn,
                    sortDesc: data.sortDesc,
                };
            }
        },
    },
});

export const {
    setAccountsSearchText,
    openNewAccountDialog,
    closeNewAccountDialog,
    openEditAccountDialog,
    closeEditAccountDialog,
    setPaginationData,
} = accountsSlice.actions;

export default accountsSlice.reducer;
