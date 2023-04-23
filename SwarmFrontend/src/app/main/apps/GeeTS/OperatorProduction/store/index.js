import { combineReducers } from "@reduxjs/toolkit";
import operatorProductions from "./operatorProductionSlice";

const reducer = combineReducers({
    operatorProductions,
});

export default reducer;
