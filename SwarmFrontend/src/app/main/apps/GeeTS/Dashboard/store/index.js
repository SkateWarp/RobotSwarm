import { combineReducers } from "@reduxjs/toolkit";
import boothDashboards from "./boothDashboardSlice";

const reducer = combineReducers({
    boothDashboards,
});

export default reducer;
