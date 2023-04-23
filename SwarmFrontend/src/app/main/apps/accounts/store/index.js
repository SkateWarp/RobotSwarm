import { combineReducers } from '@reduxjs/toolkit';
import accounts from './accountsSlice';
import user from './userSlice';

const reducer = combineReducers({
  accounts,
  user,
});

export default reducer;
