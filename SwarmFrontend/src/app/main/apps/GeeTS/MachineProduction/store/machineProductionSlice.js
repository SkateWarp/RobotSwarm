import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";

export const getMachineProductions = createAsyncThunk(
    "machineProductionApp/machineProductions/getMachineProductions",
    async (form) => {
        const { startDate, endDate, pageNumber, pageSize, searchFilter, sortColumn, sortDesc } = form;

        const response = await axios.get(`${URL}/api/SortingProduction/machines/pagination`, {
            params: {
                startDate,
                endDate,
                pageNumber,
                pageSize,
                searchFilter,
                sortColumn,
                sortDesc,
            },
        });

        const { data } = response;

        return { data, routeParams: form };
    }
);

const machineProductionAdapter = createEntityAdapter({});

export const { selectAll: selectMachineProductions } = machineProductionAdapter.getSelectors(
    (state) => state.machineProductionApp.machineProductions
);

const machineProductionSlice = createSlice({
    name: "machineProductionApp/machineProductions",
    initialState: machineProductionAdapter.getInitialState({
        searchText: "",
        routeParams: {},
        productionSorting: [],
        pagination: {
            startDate: "",
            endDate: "",
            pageSize: 0,
            pageNumber: 0,
            totalPages: 0,
            totalRecords: 0,
            searchFilter: "",
            sortColumn: "",
            sortDesc: false,
        },
    }),
    reducers: {
        setMachineProductionSearchText: {
            reducer: (state, action) => {
                state.searchText = action.payload;
            },
            prepare: (event) => ({ payload: event.target.value || "" }),
        },
    },
    extraReducers: {
        [getMachineProductions.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;
            state.productionSorting = data;
            state.routeParams = routeParams;
            state.pagination = {
                pageNumber: data.pageNumber,
                pageSize: data.pageSize,
                totalPages: data.totalPages,
                totalRecords: data.totalRecords,
                startDate: routeParams.startDate,
                endDate: routeParams.endDate,
                searchFilter: data.searchFilter,
                sortColumn: data.sortColumn,
                sortDesc: data.sortDesc,
            };
        },
    },
});

export const { setMachineProductionSearchText } = machineProductionSlice.actions;

export default machineProductionSlice.reducer;
