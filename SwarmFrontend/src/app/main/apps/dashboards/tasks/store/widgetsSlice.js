import { createEntityAdapter, createSlice, createAsyncThunk } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "../../../../../constants/constants";
import jwtService from "../../../../../services/jwtService";

export const getWidgets = createAsyncThunk("taskDashboardApp/widgets/getWidgets", async () => {
    const response = await axios.get(`${URL}/api/TaskForm/widget`, {
        headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${jwtService.getAccessToken()}`,
        },
    });
    const data = await response.data;

    return data;
});

const widgetsAdapter = createEntityAdapter({});

export const { selectEntities: selectWidgets, selectById: selectWidgetById } = widgetsAdapter.getSelectors(
    (state) => state.taskDashboardApp.widgets
);

const widgetsSlice = createSlice({
    name: "taskDashboardApp/widgets",
    initialState: widgetsAdapter.getInitialState(),
    reducers: {},
    extraReducers: {
        [getWidgets.fulfilled]: widgetsAdapter.setAll,
    },
});

export default widgetsSlice.reducer;
