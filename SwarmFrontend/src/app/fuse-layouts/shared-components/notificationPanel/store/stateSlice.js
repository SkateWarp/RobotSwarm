import { createSlice } from '@reduxjs/toolkit';

const stateSlice = createSlice({
	name: 'notificationPanel/state',
	initialState: false,
	reducers: {
		toggleNotificationPanel: (state) => !state,
		openNotificationPanel: () => true,
		closeNotificationPanel: () => false
	}
});

export const { toggleNotificationPanel, openNotificationPanel, closeNotificationPanel } = stateSlice.actions;

export default stateSlice.reducer;
