import { combineReducers } from "@reduxjs/toolkit";
import machineProductions from "./machineProductionSlice";

const reducer = combineReducers({
    machineProductions,
});

export default reducer;
