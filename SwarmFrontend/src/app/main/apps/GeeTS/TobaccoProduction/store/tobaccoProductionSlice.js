import { createSlice, createAsyncThunk, createEntityAdapter } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "app/constants/constants";

export const getTobaccoProductions = createAsyncThunk(
    "sortingProductionApp/sortingProductions/getSortingProductions",
    async (form) => {
        const { startDate, endDate, pageNumber, pageSize, searchFilter, sortColumn, sortDesc, shiftId } = form;

        const response = await axios.get(`${URL}/api/SortingProduction/shifts/pagination`, {
            params: {
                startDate,
                endDate,
                shiftId,
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

const tobaccoProductionAdapter = createEntityAdapter({});

export const { selectAll: selectTobaccoProductions } = tobaccoProductionAdapter.getSelectors(
    (state) => state.tobaccoProductionApp.tobaccoProductions
);

const tobaccoProductionSlice = createSlice({
    name: "tobaccoProductionApp/tobaccoProductions",
    initialState: tobaccoProductionAdapter.getInitialState({
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
            shiftId: 0,
            sortColumn: "",
            sortDesc: false,
        },
    }),

    extraReducers: {
        [getTobaccoProductions.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;

            // Si tengo elementos que tienen el mismo id debido a que vienen agrupados desde el backend utilizar
            // setAll me dara problemas.
            // tobaccoProductionAdapter.setAll(state, data.data);
            state.productionSorting = data;
            state.routeParams = routeParams;
            state.pagination = {
                pageNumber: data.pageNumber,
                pageSize: data.pageSize,
                totalPages: data.totalPages,
                totalRecords: data.totalRecords,
                startDate: routeParams.startDate,
                endDate: routeParams.endDate,
                shiftId: routeParams.shiftId,
                searchFilter: data.searchFilter,
                sortColumn: data.sortColumn,
                sortDesc: data.sortDesc,
            };
        },
    },
});

export default tobaccoProductionSlice.reducer;
