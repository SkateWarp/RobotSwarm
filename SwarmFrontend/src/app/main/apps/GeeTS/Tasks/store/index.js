import { combineReducers } from "@reduxjs/toolkit";
import taskConfig from "./taskConfigSlice";

const reducer = combineReducers({
    taskConfig,
});

export default reducer;
