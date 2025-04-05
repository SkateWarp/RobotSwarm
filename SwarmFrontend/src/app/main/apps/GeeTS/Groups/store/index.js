import { combineReducers } from "@reduxjs/toolkit";
import leafSorting from "./leafSortingConfigSlice";

const reducer = combineReducers({
    leafSorting,
});

export default reducer;
