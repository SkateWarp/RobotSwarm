import { combineReducers } from "@reduxjs/toolkit";
import tobaccoProductions from "./tobaccoProductionSlice";

const reducer = combineReducers({
    tobaccoProductions,
});

export default reducer;
