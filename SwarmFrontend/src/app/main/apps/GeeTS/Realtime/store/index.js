import { combineReducers } from "@reduxjs/toolkit";
import config from "./realtimeConfigSlice";

const reducer = combineReducers({
    config,
});

export default reducer;
