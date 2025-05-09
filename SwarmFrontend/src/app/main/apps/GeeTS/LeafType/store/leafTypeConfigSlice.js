import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";

export const getLeafTypes = createAsyncThunk(
    "leafTypesConfigApp/leafType-components/getLeafTypes",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().leafTypesConfigApp.leafTypes.routeParams;

        const response = await axios.get(`${URL}/TaskLog`);
        const data = await response.data;

        return { data, routeParams };
    }
);

export const addNewLeafType = createAsyncThunk(
    "leafTypesConfigApp/leafType-components/addNewLeafType",
    async (newLeafType, { dispatch }) => {
        const response = await axios.post(`${URL}/api/LeafType`, newLeafType);
        const data = await response.data;

        dispatch(getLeafTypes());

        return data;
    }
);

export const updateLeafType = createAsyncThunk(
    "leafTypesConfigApp/leafType-components/updateLeafType",
    async (leafType, { dispatch }) => {
        const response = await axios.put(`${URL}/api/LeafType`, leafType);
        const data = await response.data;

        dispatch(getLeafTypes());

        return data;
    }
);

export const removeLeafType = createAsyncThunk(
    "leafTypesConfigApp/leafType-components/removeLeafType",
    async (idForDelete, { dispatch }) => {
        const response = await axios.put(`${URL}/api/LeafType/disable/${idForDelete}`);
        const data = await response.data;
        dispatch(getLeafTypes());

        return data;
    }
);

const leafTypeConfigAdapter = createEntityAdapter({});

export const { selectAll: selectLeafTypes } = leafTypeConfigAdapter.getSelectors(
    (state) => state.leafTypesConfigApp.leafTypes
);

const leafTypesConfigSlice = createSlice({
    name: "leafTypesConfigApp/leafTypes",
    initialState: leafTypeConfigAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        leafTypesConfigs: [],
        leafTypesConfigDialog: {
            type: "new",
            props: {
                open: false,
            },
            data: null,
        },
    }),
    reducers: {
        openNewLeafTypesConfigDialog: (state) => {
            state.leafTypesConfigDialog = {
                type: "new",
                props: {
                    open: true,
                },
                data: null,
            };
        },

        closeNewLeafTypesConfigDialog: (state) => {
            state.leafTypesConfigDialog = {
                type: "new",
                props: {
                    open: false,
                },
                data: null,
            };
        },

        openEditLeafTypesConfigDialog: (state, action) => {
            state.leafTypesConfigDialog = {
                type: "edit",
                props: {
                    open: true,
                },
                data: action.payload,
            };
        },
        closeEditLeafTypesConfigDialog: (state) => {
            state.leafTypesConfigDialog = {
                type: "edit",
                props: {
                    open: false,
                },
                data: null,
            };
        },
    },

    extraReducers: {
        [getLeafTypes.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;

            leafTypeConfigAdapter.setAll(state, data);

            state.leafTypesConfigs = data;
            state.routeParams = routeParams;
        },
    },
});

export const {
    openNewLeafTypesConfigDialog,
    closeNewLeafTypesConfigDialog,
    openEditLeafTypesConfigDialog,
    closeEditLeafTypesConfigDialog,
} = leafTypesConfigSlice.actions;

export default leafTypesConfigSlice.reducer;
