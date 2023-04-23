import { createAsyncThunk, createEntityAdapter, createSlice } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "../../../../../constants/constants";

export const getStopCause = createAsyncThunk(
    "boothDashboardDetailsApp/boothDashboards/getStopCause",
    async (form) => {
        const {
            startDate,
            endDate,
            operatorId,
            machineId,
            alarmType,
            pageSize,
            pageNumber,
            searchFilter,
            alarmTypes,
            sortColumn,
            sortDesc,
        } = form;

        const response = await axios.get(`${URL}/api/Alarm/stopCauseFilter`, {
            params: {
                startDate,
                endDate,
                operatorId,
                alarmType,
                pageNumber,
                pageSize,
                searchFilter,
                StopAlarm: true,
                sortColumn,
                sortDesc,
                machineId,
                alarmTypes,
            },
        });

        const data = await response.data;

        return { data, routeParams: form };
    }
);

export const getChartStops = createAsyncThunk(
    "boothDashboardDetailsApp/boothDashboards/getChartStops",
    async (form) => {
        const { startDate, endDate, stopType, machineId, operatorId, alarmTypes } = form;

        const response = await axios.get(`${URL}/api/Alarm/chart`, {
            params: {
                start: startDate,
                end: endDate,
                stopType,
                StopAlarm: true,
                machineId,
                operatorId,
                alarmTypes,
            },
        });
        const data = await response.data;

        return { data, routeParams: form };
    }
);

const dashboardAdapter = createEntityAdapter({});

export const { selectAll: selectStops } = dashboardAdapter.getSelectors(
    (state) => state.boothDashboardDetailsApp.boothDashboards
);

const boothDashboardSlice = createSlice({
    name: "boothDashboardDetailsApp/boothDashboards",
    initialState: dashboardAdapter.getInitialState({
        stops: [],
        report: [],
        machines: [],
        routeParams: {},
        searchText: "",
        pagination: {
            startDate: "",
            endDate: "",
            operatorId: 0,
            pageSize: 5,
            pageNumber: 1,
            totalPages: 0,
            totalRecords: 0,
            searchFilter: "",
            sortColumn: "",
            sortDesc: false,
        },
    }),

    extraReducers: {
        [getStopCause.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;
            state.stops = data.data;
            dashboardAdapter.setAll(state, data.data);
            state.routeParams = routeParams;
            state.pagination = {
                pageNumber: data.pageNumber,
                pageSize: data.pageSize,
                totalPages: data.totalPages,
                totalRecords: data.totalRecords,
                startDate: routeParams.startDate,
                operatorId: routeParams.operatorId,
                machineId: routeParams.machineId,
                endDate: routeParams.endDate,
                alarmType: routeParams.alarmType,
                alarmTypes: routeParams.alarmTypes,
                searchFilter: data.searchFilter,
                sortColumn: data.sortColumn,
                sortDesc: data.sortDesc,
            };
        },

        [getChartStops.fulfilled]: (state, action) => {
            const { data } = action.payload;
            state.report = data;
        },
    },
});

export default boothDashboardSlice.reducer;
