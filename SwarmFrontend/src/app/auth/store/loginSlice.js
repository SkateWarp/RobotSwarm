import { createSlice } from "@reduxjs/toolkit";
import jwtService from "../../services/jwtService";
import { setUserData } from "./userSlice";
import { showMessage } from "../../store/fuse/messageSlice";

export const submitLogin =
    ({ email, password }) =>
    async (dispatch) => {
        return jwtService
            .signInWithEmailAndPassword(email, password)
            .then((user) => {
                dispatch(setUserData(user));
                user = null;
                dispatch(
                    showMessage({
                        message: "Bienvenido", // text or html
                        autoHideDuration: 3000, // ms
                        anchorOrigin: {
                            vertical: "top", // top bottom
                            horizontal: "center", // left center right
                        },
                        variant: "success", // success error info warning null
                    })
                );
                return dispatch(loginSuccess());
            })
            .catch((error) => {
                dispatch(
                    showMessage({
                        message: error.message, // text or html
                        autoHideDuration: 3000, // ms
                        anchorOrigin: {
                            vertical: "top", // top bottom
                            horizontal: "center", // left center right
                        },
                        variant: "error", // success error info warning null
                    })
                );
                return dispatch(loginError(error));
            });
    };

const initialState = {
    success: false,
    error: {
        username: null,
        password: null,
    },
};

const loginSlice = createSlice({
    name: "auth/login",
    initialState,
    reducers: {
        loginSuccess: (state, action) => {
            state.success = true;
        },
        loginError: (state, action) => {
            state.success = false;
            state.error = action.payload;
        },
    },
    extraReducers: {},
});

export const { loginSuccess, loginError } = loginSlice.actions;

export default loginSlice.reducer;
