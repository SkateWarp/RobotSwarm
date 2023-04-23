import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";

export const getOperatorProductions = createAsyncThunk(
    "operatorProductionApp/operatorProductions/getOperatorProductions",
    async (form) => {
        const { startDate, endDate, operatorId, pageNumber, pageSize, searchFilter, sortColumn, sortDesc } =
            form;

        const response = await axios.get(`${URL}/api/SortingProduction/operators/pagination`, {
            params: {
                startDate,
                endDate,
                operatorId,
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

const operatorProductionAdapter = createEntityAdapter({});

export const { selectAll: selectOperatorProductions } = operatorProductionAdapter.getSelectors(
    (state) => state.operatorProductionApp.operatorProductions
);

const operatorProductionSlice = createSlice({
    name: "operatorProductionApp/operatorProductions",
    initialState: operatorProductionAdapter.getInitialState({
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
            operatorId: 0,
            sortColumn: "",
            sortDesc: false,
        },
    }),

    extraReducers: {
        [getOperatorProductions.fulfilled]: (state, action) => {
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
                operatorId: routeParams.operatorId,
                searchFilter: data.searchFilter,
                sortColumn: data.sortColumn,
                sortDesc: data.sortDesc,
            };
        },
    },
});

export default operatorProductionSlice.reducer;
