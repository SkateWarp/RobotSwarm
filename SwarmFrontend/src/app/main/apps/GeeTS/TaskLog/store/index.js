import { combineReducers } from "@reduxjs/toolkit";
import taskLogs from "./taskLogSlice";

const reducer = combineReducers({
    taskLogs,
});

export default reducer;
