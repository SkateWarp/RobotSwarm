import {createAsyncThunk, createSlice} from '@reduxjs/toolkit';
import axios from 'axios';
import {URL} from 'app/constants/constants';


export const getUserData = createAsyncThunk('accountsApp/user/getUserData', async () => {
    const response = await axios.get(`${URL}/api/Accounts`);
    return await response.data;
});

const userSlice = createSlice({
    name: 'accountsApp/user',
    initialState: {},
    reducers: {},
    extraReducers: {
        [getUserData.fulfilled]: (state, action) => action.payload,
    },
});

export default userSlice.reducer;
