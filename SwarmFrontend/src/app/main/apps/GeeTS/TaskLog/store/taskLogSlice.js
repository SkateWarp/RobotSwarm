import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";
import jwtService from "app/services/jwtService";

export const getTaskLogs = createAsyncThunk(
    "taskLogApp/taskLogs/getTaskLogs",
    async (routeParams, { getState }) => {
        routeParams = routeParams || getState().taskLogApp.taskLogs.routeParams;

        const response = await axios.get(`${URL}/TaskLog`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        return { data, routeParams };
    }
);

export const getTaskLogById = createAsyncThunk(
    "taskLogApp/taskLogs/getTaskLogById",
    async (taskLogId, { getState }) => {
        const response = await axios.get(`${URL}/api/TaskLog/${taskLogId}`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        return data;
    }
);

export const finishTaskLog = createAsyncThunk(
    "taskLogApp/taskLogs/finishTaskLog",
    async (robotId, { dispatch }) => {
        const response = await axios.put(`${URL}/api/TaskLog/finish/${robotId}`, {}, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        dispatch(getTaskLogs());

        return data;
    }
);

export const cancelTaskLog = createAsyncThunk(
    "taskLogApp/taskLogs/cancelTaskLog",
    async (robotId, { dispatch }) => {
        const response = await axios.put(`${URL}/api/TaskLog/cancel/${robotId}`, {}, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        });
        const data = await response.data;

        dispatch(getTaskLogs());

        return data;
    }
);

const taskLogAdapter = createEntityAdapter({});

export const { selectAll: selectTaskLogs, selectById: selectTaskLogById } = taskLogAdapter.getSelectors(
    (state) => state.taskLogApp.taskLogs
);

const taskLogSlice = createSlice({
    name: "taskLogApp/taskLogs",
    initialState: taskLogAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        taskLogs: [],
        taskLogDialog: {
            type: "view",
            props: {
                open: false,
            },
            data: null,
        },
        selectedTaskLog: null,
    }),
    reducers: {
        setTaskLogsSearchText: {
            reducer: (state, action) => {
                state.searchText = action.payload;
            },
            prepare: (event) => ({ payload: event.target.value || "" }),
        },

        openTaskLogDialog: (state, action) => {
            state.taskLogDialog = {
                type: "view",
                props: {
                    open: true,
                },
                data: action.payload,
            };
        },

        closeTaskLogDialog: (state) => {
            state.taskLogDialog = {
                type: "view",
                props: {
                    open: false,
                },
                data: null,
            };
        },

        setSelectedTaskLog: (state, action) => {
            state.selectedTaskLog = action.payload;
        },

        clearSelectedTaskLog: (state) => {
            state.selectedTaskLog = null;
        },
    },

    extraReducers: {
        [getTaskLogs.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;

            taskLogAdapter.setAll(state, data);

            state.taskLogs = data;
            state.routeParams = routeParams;
        },

        [getTaskLogById.fulfilled]: (state, action) => {
            const taskLog = action.payload;
            taskLogAdapter.upsertOne(state, taskLog);
        },

        [finishTaskLog.fulfilled]: (state, action) => {
            const updatedTaskLog = action.payload;
            taskLogAdapter.upsertOne(state, updatedTaskLog);
        },

        [cancelTaskLog.fulfilled]: (state, action) => {
            const updatedTaskLog = action.payload;
            taskLogAdapter.upsertOne(state, updatedTaskLog);
        },
    },
});

export const {
    setTaskLogsSearchText,
    openTaskLogDialog,
    closeTaskLogDialog,
    setSelectedTaskLog,
    clearSelectedTaskLog,
} = taskLogSlice.actions;

export default taskLogSlice.reducer;
