import { createSlice } from '@reduxjs/toolkit';
import jwtService from 'app/services/jwtService';
import { setUserData } from './userSlice';

export const submitRegister =
  ({ displayName, password, email }) =>
    async (dispatch) => {
      return jwtService
        .createUser({
          displayName,
          password,
          email,
        })
        .then((user) => {
          dispatch(setUserData(user));
          return dispatch(registerSuccess());
        })
        .catch((errors) => {
          return dispatch(registerError(errors));
        });
    };

const initialState = {
  success: false,
  errors: [],
};

const registerSlice = createSlice({
  name: 'auth/register',
  initialState,
  reducers: {
    registerSuccess: (state, action) => {
      state.success = true;
      state.errors = [];
    },
    registerError: (state, action) => {
      state.success = false;
      state.errors = action.payload;
    },
  },
  extraReducers: {},
});

export const { registerSuccess, registerError } = registerSlice.actions;

export default registerSlice.reducer;
