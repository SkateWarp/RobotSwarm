import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "../../../../../services/jwtService";

export const getLeafSorting = createAsyncThunk(
    "leafSortingConfigApp/leafSorting/getLeafSorting",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().leafSortingConfigApp.leafSorting.routeParams;

        const response = await axios.get(`${URL}/Robots`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        return { data, routeParams };
    }
);

export const addNewLeafSorting = createAsyncThunk(
    "leafSortingConfigApp/leafSorting/addNewLeafSorting",
    async (newRobot, { dispatch }) => {
        const response = await axios.post(`${URL}/Robots`, newRobot, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        dispatch(getLeafSorting());

        return data;
    }
);

export const updateLeafSorting = createAsyncThunk(
    "leafSortingConfigApp/leafSorting/updateLeafSorting",
    async (leafSorting, { dispatch }) => {
        const response = await axios.put(`${URL}/api/LeafSorting`, leafSorting);
        const data = await response.data;

        dispatch(getLeafSorting());

        return data;
    }
);

export const removeLeafSorting = createAsyncThunk(
    "leafSortingConfigApp/leafSorting/removeLeafSorting",
    async (idForDelete, { dispatch }) => {
        const response = await axios.put(`${URL}/api/LeafSorting/disable/${idForDelete}`);
        const data = await response.data;
        dispatch(getLeafSorting());

        return data;
    }
);

const leafSortingConfigAdapter = createEntityAdapter({});

export const { selectAll: selectLeafSorting } = leafSortingConfigAdapter.getSelectors(
    (state) => state.leafSortingConfigApp.leafSorting
);

const leafSortingConfigSlice = createSlice({
    name: "leafSortingConfigApp/leafSorting",
    initialState: leafSortingConfigAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        leafSortingConfigs: [],
        leafSortingConfigDialog: {
            type: "new",
            props: {
                open: false,
            },
            data: null,
        },
    }),
    reducers: {
        openNewLeafSortingConfigDialog: (state) => {
            state.leafSortingConfigDialog = {
                type: "new",
                props: {
                    open: true,
                },
                data: null,
            };
        },

        closeNewLeafSortingConfigDialog: (state) => {
            state.leafSortingConfigDialog = {
                type: "new",
                props: {
                    open: false,
                },
                data: null,
            };
        },

        openEditLeafSortingConfigDialog: (state, action) => {
            state.leafSortingConfigDialog = {
                type: "edit",
                props: {
                    open: true,
                },
                data: action.payload,
            };
        },
        closeEditLeafSortingConfigDialog: (state) => {
            state.leafSortingConfigDialog = {
                type: "edit",
                props: {
                    open: false,
                },
                data: null,
            };
        },
    },

    extraReducers: {
        [getLeafSorting.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;

            leafSortingConfigAdapter.setAll(state, data);

            state.leafSortingConfigs = data;
            state.routeParams = routeParams;
        },
    },
});

export const {
    openNewLeafSortingConfigDialog,
    closeNewLeafSortingConfigDialog,
    openEditLeafSortingConfigDialog,
    closeEditLeafSortingConfigDialog,
} = leafSortingConfigSlice.actions;

export default leafSortingConfigSlice.reducer;
