import { combineReducers } from "@reduxjs/toolkit";
import leafTypes from "./leafTypeConfigSlice";

const reducer = combineReducers({
    leafTypes,
});

export default reducer;
