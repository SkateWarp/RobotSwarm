import { createSlice, createAsyncThunk, createEntityAdapter } from '@reduxjs/toolkit';
import axios from 'axios';
import { URL } from 'app/constants/constants';
import { getAccounts } from './accountsSlice';
import jwtService from '../../../../services/jwtService';
import { showMessage } from '../../../../store/fuse/messageSlice';
import { registerError, registerSuccess } from '../../../../auth/store/registerSlice';

export const submitOperator = (model) => async dispatch => {
    return jwtService
        .createOperator(model)
        .then(user => {
            return dispatch(registerSuccess(user));
        })
        .catch(error => {
            if(error.message!==undefined) {
                dispatch(showMessage({
                    message: error.message,//text or html
                    autoHideDuration: 3000,//ms
                    anchorOrigin: {
                        vertical: 'top',//top bottom
                        horizontal: 'center'//left center right
                    },
                    variant: 'error'//success error info warning null
                }));
            }
            return dispatch(registerError(error));
        });
};

export const addOperator = createAsyncThunk(
    'accountsApp/accounts/addOperator',
    async (operator, { dispatch }) => {
        const response = await 	axios.post(`${URL}/api/Accounts/operator`, operator);
        const data = await response.data;

        dispatch(getAccounts());

        return data;
    }
);

const operatorAdapter = createEntityAdapter({});

export const { selectAll: selectAccounts, selectById: selectAccountsById } = operatorAdapter.getSelectors(
    state => state.accountsApp.operator
);

const operatorSlice = createSlice({
    name: 'accountsApp/operator',
    initialState: operatorAdapter.getInitialState({
        searchText: '',
        routeParams: {},
        accountDialog: {
            type: 'new',
            props: {
                open: false
            },
            data: null
        }
    }),
    reducers: {},
    extraReducers: {
        //[updateAccount.fulfilled]: accountsAdapter.upsertOne,
        [addOperator.fulfilled]: operatorAdapter.addOne,
        /*[getAccounts.fulfilled]: (state, action) => {
            const { data, routeParams } = action.payload;
            accountsAdapter.setAll(state, data);
            state.routeParams = routeParams;
            state.searchText = '';
        }*/
    }
});

export default operatorSlice.reducer;
