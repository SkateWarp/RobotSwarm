import { createAsyncThunk, createEntityAdapter, createSlice } from "@reduxjs/toolkit";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import useActualProjectName from "../../../../shared-components/hooks/useActualProjectName";

// eslint-disable-next-line react-hooks/rules-of-hooks
const actualProjectName = useActualProjectName();

export const getMachinesStop = createAsyncThunk("stopTimeMachine/stopTime/getMachinesStop", async () => {
    const response = await axios.get(`${URL}/api/MechanicCall`);

    const data = await response.data;

    return { data };
});

export const getPendingMaintenances = createAsyncThunk("quickPanel/getPendingMaintenance", async () => {
    const response = await axios.get(`${URL}/api/MachineMaintenanceLog/Line`, {
        params: { percent: 0.9 },
    });
    const data = await response.data;

    return { data };
});

export const getOngoingAlarms = createAsyncThunk("quickPanel/getOngoingAlarms", async () => {
    const response = await axios.get(`${URL}/api/Alarm/Line`, {
        params: { limit: 20 },
    });
    const data = await response.data;

    return { data };
});

export const getLines = createAsyncThunk("quickPanel/getLines", async () => {
    const response = await axios.get(`${URL}/api/LineProduction`);
    const data = await response.data;

    return { data };
});

const stateAdapter = createEntityAdapter({});

export const { selectAll: selectMechanicCalls } = stateAdapter.getSelectors((state) => state.quickPanel.state);

const stateSlice = createSlice({
    name: "quickPanel/state",
    initialState: {
        state: false,
        lines: [],
        lineAlarms: [],
        maintenancePerMachine: [],
        mechanicCalls: [],
        alarmsPerMachine: {},
        totalNotifications: 0,
    },
    reducers: {
        toggleQuickPanel: (state) => {
            state.state = !state.state;
        },
        openQuickPanel: (state) => {
            state.state = true;
        },
        closeQuickPanel: (state) => {
            state.state = false;
        },

        addMechanicCalls: (state, action) => {
            state.mechanicCalls = action.payload;
        },

        clearAlarms: (state, action) => {
            const { lineId, machineId } = action.payload;

            if (state.alarmsPerMachine[lineId]) {
                state.alarmsPerMachine[lineId][machineId] = 0;
            }
        },
        clearMaintenance: (state, action) => {
            const { lineId, machineId } = action.payload;

            if (state.maintenancePerMachine[lineId]) {
                state.maintenancePerMachine[lineId][machineId] = 0;
            }
        },
        setOngoingAlarms: (state, action) => {
            const { data } = action.payload;

            // Borrar las alarmas anteriores de la línea
            state.lineAlarms
                .filter((x) => x.id === data.line)
                .forEach((x) => state.lineAlarms.splice(state.lineAlarms.indexOf(x), 1));

            // Agregar alarmas nuevas de la línea
            if (actualProjectName === "baldom")
                this.state.lineAlarms = [...this.state.lineAlarms, data.alarms];

            // Contar todas las alarmas
            let alarmsCount = 0;
            state.lineAlarms.forEach((line) => {
                const lineAlarmas = line.machines.reduce(
                    (totalA, machine) => totalA + machine.alarms.length,
                    0
                );
                alarmsCount += lineAlarmas;
            });

            state.totalNotifications = alarmsCount;
        },
        addAlarmLength: (state, action) => {
            const { lineId, machineId, alarms } = action.payload;
            if (state.alarmsPerMachine[lineId]) {
                state.alarmsPerMachine[lineId][machineId] = alarms;
            } else {
                state.alarmsPerMachine[lineId] = { [machineId]: alarms };
            }
        },
        addMaintenanceLength: (state, action) => {
            const { lineId, machineId, maintenance } = action.payload;
            if (state.maintenancePerMachine[lineId]) {
                state.maintenancePerMachine[lineId][machineId] = maintenance;
            } else {
                state.maintenancePerMachine[lineId] = { [machineId]: maintenance };
            }
        },
    },
    extraReducers: {
        [getPendingMaintenances.fulfilled]: (state, action) => {
            const { data } = action.payload;
            state.lineMaintenance = data;
        },
        [getOngoingAlarms.fulfilled]: (state, action) => {
            const { data } = action.payload;
            state.lineAlarms = data;

            let alarmsCount = 0;
            data.forEach((line) => {
                // Agregado ? para evitar error undefined en consola
                const lineAlarmas = line?.machines?.reduce(
                    (totalA, machine) => totalA + machine.alarms.length,
                    0
                );
                alarmsCount += lineAlarmas;
            });

            state.totalNotifications = alarmsCount;
        },
        [getLines.fulfilled]: (state, action) => {
            const { data } = action.payload;
            state.lines = data;
        },

        [getMachinesStop.fulfilled]: (state, action) => {
            const { data } = action.payload;
            state.mechanicCalls = data;
        },
    },
});

export const {
    toggleQuickPanel,
    openQuickPanel,
    setOngoingAlarms,
    addMaintenanceLength,
    addAlarmLength,
    clearAlarms,
    clearMaintenance,
    addMechanicCalls,
} = stateSlice.actions;

export default stateSlice.reducer;
