import { createAsyncThunk, createSlice } from '@reduxjs/toolkit';
import axios from 'axios';
import { URL } from '../../../../constants/constants';

export const getPendingMaintenances = createAsyncThunk('quickPanel/getPendingMaintenance', async machineId => {
	const response = await axios.get(`${URL}/api/MachineMaintenanceLog/percentage/machine/${machineId}?percentage=1`);
	const data = await response.data;

	return { data, machineId };
});

export const getOngoingAlarms = createAsyncThunk('quickPanel/getOngoingAlarms', async machineId => {
	const response = await axios.get(`${URL}/api/Alarm/machine/${machineId}`, {
		params: { limit: 20, finished: false }
	});
	const data = await response.data;

	return { data, machineId };
});

const stateSlice = createSlice({
	name: 'quickPanel/state',
	initialState: {
		state: false,
		slitterAlarms: [],
		tubeMillAlarms: [],
		tolaAlarms: [],
		aluzincAlarms: [],
		slitterMaintenances: [],
		tubeMillMaintenances: [],
		tolaMaintenances: [],
		aluzincMaintenances: []
	},
	reducers: {
		toggleQuickPanelFraga: (state) => {
			state.state = !state.state;
		},
		openQuickPanelFraga: (state) => {
			state.state = true;
		},
		closeQuickPanelFraga: (state) => {
			state.state = false;
		},
		setPendingMaintenances: (state, action) => {
			const { maintenance, machineId } = action.payload;

			if (machineId === 1) {
				state.slitterMaintenances = maintenance;
			} else if (machineId === 2) {
				state.tubeMillMaintenances = maintenance;
			} else if (machineId === 4) {
				state.aluzincMaintenances = maintenance;
			} else if (machineId === 5) {
				state.tolaMaintenances = maintenance;
			}
		},
		setOngoingAlarms: (state, action) => {
			const { alarms, machineId } = action.payload;
			if (machineId === 1) {
				state.slitterAlarms = alarms;
			} else if (machineId === 2) {
				state.tubeMillAlarms = alarms;
			} else if (machineId === 4) {
				state.aluzincAlarms = alarms;
			} else if (machineId === 5) {
				state.tolaAlarms = alarms;
			}
		}
	},
	extraReducers: {
		// [getMetalTypes.fulfilled]: metalTypeAdapter.setAll,
		[getPendingMaintenances.fulfilled]: (state, action) => {
			const { data, machineId } = action.payload;

			if (machineId === 1) {
				state.slitterMaintenances = data;
			} else if (machineId === 2) {
				state.tubeMillMaintenances = data;
			} else if (machineId === 4) {
				state.aluzincMaintenances = data;
			} else if (machineId === 5) {
				state.tolaMaintenances = data;
			}
		},
		[getOngoingAlarms.fulfilled]: (state, action) => {
			const { data, machineId } = action.payload;

			if (machineId === 1) {
				state.slitterAlarms = data;
			} else if (machineId === 2) {
				state.tubeMillAlarms = data;
			} else if (machineId === 4) {
				state.aluzincAlarms = data;
			} else if (machineId === 5) {
				state.tolaAlarms = data;
			}
		}
	}
});

export const {
	toggleQuickPanelFraga,
	openQuickPanelFraga,
	closeQuickPanelFraga,
	setOngoingAlarms,
	setPendingMaintenances
} = stateSlice.actions;

export default stateSlice.reducer;
