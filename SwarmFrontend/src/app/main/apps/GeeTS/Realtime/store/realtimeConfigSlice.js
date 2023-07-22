import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";

export const getTasks = createAsyncThunk(
    "realtimeConfigApp/config/getTasks",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().realtimeConfigApp.taskConfig.routeParams;
        const response = await axios.get(`${URL}/api/TaskActivity`);
        const data = await response.data;

        return { data, routeParams };
    }
);

export const addNewTaskType = createAsyncThunk(
    "realtimeConfigApp/taskConfig/addNewTaskType",
    async (taskType, { dispatch }) => {
        const response = await axios.post(`${URL}/api/TaskActivity`, taskType);
        const data = await response.data;

        dispatch(getTasks());

        return data;
    }
);

export const updateTaskType = createAsyncThunk(
    "realtimeConfigApp/taskConfig/updateTaskType",
    async (taskType, { dispatch }) => {
        const response = await axios.put(`${URL}/api/TaskActivity/${taskType.id}`, taskType);
        const data = await response.data;

        dispatch(getTasks());

        return data;
    }
);

export const removeTaskType = createAsyncThunk(
    "realtimeConfigApp/taskConfig/removeTaskType",
    async (idForDelete, { dispatch }) => {
        const response = await axios.delete(`${URL}/api/TaskActivity/${idForDelete}`);
        const data = await response.data;
        dispatch(getTasks());

        return data;
    }
);

const taskConfigAdapter = createEntityAdapter({});

export const { selectAll: selectTaskType } = taskConfigAdapter.getSelectors(
    (state) => state.realtimeConfigApp.config
);

const taskConfigSlice = createSlice({
    name: "realtimeConfigApp/config",
    initialState: taskConfigAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        params: "",
        configs: [],
        configDialog: {
            type: "new",
            props: {
                open: false,
            },
            data: null,
        },
    }),

    reducers: {
        setTaskConfigSearchText: {
            reducer: (state, action) => {
                state.searchText = action.payload;
            },
            prepare: (event) => ({ payload: event.target.value || "" }),
        },

        openNewTaskConfigDialog: (state) => {
            state.configDialog = {
                type: "new",
                props: {
                    open: true,
                },
                data: null,
            };
        },

        closeNewTaskConfigDialog: (state) => {
            state.configDialog = {
                type: "new",
                props: {
                    open: false,
                },
                data: null,
            };
        },

        openEditTaskConfigDialog: (state, action) => {
            state.configDialog = {
                type: "edit",
                props: {
                    open: true,
                },
                data: action.payload,
            };
        },
        closeEditTaskConfigDialog: (state) => {
            state.configDialog = {
                type: "edit",
                props: {
                    open: false,
                },
                data: null,
            };
        },
    },

    extraReducers: {
        [getTasks.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;

            taskConfigAdapter.setAll(state, data);

            state.configs = data;
            state.routeParams = routeParams;
        },
    },
});

export const {
    setTaskConfigSearchText,
    openNewTaskConfigDialog,
    closeNewTaskConfigDialog,
    openEditTaskConfigDialog,
    closeEditTaskConfigDialog,
} = taskConfigSlice.actions;

export default taskConfigSlice.reducer;
