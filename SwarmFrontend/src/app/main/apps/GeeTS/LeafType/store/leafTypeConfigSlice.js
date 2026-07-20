import { createAsyncThunk, createEntityAdapter, createSlice } from "@reduxjs/toolkit";
import { getTaskTemplateErrorMessage, listTaskTemplates, saveTaskTemplate } from "../taskTemplateApi";

export const getTaskTemplates = createAsyncThunk(
    "leafTypesConfigApp/taskTemplates/getTaskTemplates",
    async (_, { rejectWithValue }) => {
        try {
            return await listTaskTemplates();
        } catch (error) {
            return rejectWithValue(getTaskTemplateErrorMessage(error));
        }
    }
);

export const updateTaskTemplate = createAsyncThunk(
    "leafTypesConfigApp/taskTemplates/updateTaskTemplate",
    async (template, { rejectWithValue }) => {
        try {
            return await saveTaskTemplate(template);
        } catch (error) {
            return rejectWithValue(getTaskTemplateErrorMessage(error));
        }
    }
);

const taskTemplateAdapter = createEntityAdapter({
    sortComparer: (left, right) => left.id - right.id,
});

export const { selectAll: selectTaskTemplates } = taskTemplateAdapter.getSelectors(
    (state) => state.leafTypesConfigApp.leafTypes
);

const leafTypesConfigSlice = createSlice({
    name: "leafTypesConfigApp/taskTemplates",
    initialState: taskTemplateAdapter.getInitialState({
        loading: false,
        error: null,
        saving: false,
        saveError: null,
        editDialog: {
            open: false,
            template: null,
        },
    }),
    reducers: {
        openTaskTemplateDialog: (state, action) => {
            state.editDialog = {
                open: true,
                template: action.payload,
            };
            state.saveError = null;
        },
        closeTaskTemplateDialog: (state) => {
            state.editDialog = {
                open: false,
                template: null,
            };
            state.saveError = null;
        },
    },
    extraReducers: (builder) => {
        builder
            .addCase(getTaskTemplates.pending, (state) => {
                state.loading = true;
                state.error = null;
            })
            .addCase(getTaskTemplates.fulfilled, (state, action) => {
                state.loading = false;
                state.error = null;
                taskTemplateAdapter.setAll(state, action.payload);
            })
            .addCase(getTaskTemplates.rejected, (state, action) => {
                state.loading = false;
                state.error = action.payload || "No fue posible cargar las plantillas.";
            })
            .addCase(updateTaskTemplate.pending, (state) => {
                state.saving = true;
                state.saveError = null;
            })
            .addCase(updateTaskTemplate.fulfilled, (state, action) => {
                state.saving = false;
                state.saveError = null;
                taskTemplateAdapter.upsertOne(state, action.payload);
            })
            .addCase(updateTaskTemplate.rejected, (state, action) => {
                state.saving = false;
                state.saveError = action.payload || "No fue posible guardar la plantilla.";
            });
    },
});

export const { openTaskTemplateDialog, closeTaskTemplateDialog } = leafTypesConfigSlice.actions;

export default leafTypesConfigSlice.reducer;
