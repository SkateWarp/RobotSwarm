import DialogContent from "@mui/material/DialogContent";
import { useCallback, useEffect, useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import { AppBar, Dialog, Icon, MenuItem, TextField, Toolbar, Typography } from "@mui/material";
import * as Actions from "../../../../store/fuse/messageSlice";
import GeneralDialogActionButtons from "../../../../shared-components/GeneralDialogActionButtons";
import {
    addNewTaskLog,
    closeEditLeafTypesConfigDialog,
    closeNewLeafTypesConfigDialog,
    updateTask,
} from "./store/leafTypeConfigSlice";
import axios from "axios";
import {URL} from "../../../../constants/constants";

const defaultValues = {
    robots: "",
    parameters: "",
    taskTemplateId: 1
};

function LeafTypesConfigDialog() {
    const dispatch = useDispatch();
    const leafTypesConfigDialog = useSelector(
        ({ leafTypesConfigApp }) => leafTypesConfigApp.leafTypes.leafTypesConfigDialog
    );

    const [taskTemplates, setTaskTemplates] = useState([]);

    const {
        control,
        reset,
        handleSubmit,
        formState: { errors, isValid },
    } = useForm({
        mode: "onChange",
        defaultValues
    });

    const initDialog = useCallback(() => {

        if (leafTypesConfigDialog.type === "edit" && leafTypesConfigDialog.data) {

            reset({
                ...leafTypesConfigDialog.data
            });
        }

        if (leafTypesConfigDialog.type === "new") {
            reset({
                ...defaultValues,
                taskTemplateId: 1,
                parameters: ""
            });
        }
    }, [leafTypesConfigDialog.data]);

    useEffect(() => {

        axios.get(`${URL}/TaskTemplate`).then((response) => {
            setTaskTemplates(response.data);
        });

        if (leafTypesConfigDialog.props.open) {
            initDialog();
        }
    }, [leafTypesConfigDialog.props.open, initDialog]);

    function closeComposeDialog() {
        return leafTypesConfigDialog.type === "edit"
            ? dispatch(closeEditLeafTypesConfigDialog())
            : dispatch(closeNewLeafTypesConfigDialog());
    }

    function onSubmit(data) {
        if (leafTypesConfigDialog.type === "new") {
            dispatch(Actions.showMessage({ message: "Creando..." }));
            dispatch(addNewTaskLog(data));
        } else {
            dispatch(Actions.showMessage({ message: "Actualizando..." }));
            dispatch(updateTask(data));
        }
        closeComposeDialog();
    }

    return (
        <Dialog
            classes={{
                paper: "m-24",
            }}
            {...leafTypesConfigDialog.props}
            onClose={closeComposeDialog}
            fullWidth
            maxWidth="xs"
        >
            <AppBar position="static" elevation={0}>
                <Toolbar className="flex w-full">
                    <Typography variant="subtitle1" color="inherit">
                        {leafTypesConfigDialog.type === "new" ? "Nuevo" : "Editar"}
                    </Typography>
                </Toolbar>
            </AppBar>
            <form noValidate onSubmit={handleSubmit(onSubmit)} className="flex flex-col md:overflow-hidden">
                <DialogContent classes={{ root: "p-24" }}>
                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">account_circle</Icon>
                        </div>
                        <Controller
                            control={control}
                            name="parameters"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Parametros"
                                    type="text"
                                    error={!!errors.name}
                                    helperText={errors?.name?.message}
                                    variant="outlined"
                                    required
                                    fullWidth
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">category</Icon>
                        </div>
                        <Controller
                            control={control}
                            name="robots"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Robots"
                                    variant="outlined"
                                    type="text"
                                    fullWidth
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">category</Icon>
                        </div>

                        <Controller
                            name="taskTemplateId"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    autoFocus
                                    select
                                    margin="dense"
                                    label="Plantilla"
                                    className="w-full"
                                    required
                                >
                                    {taskTemplates.map((taskTemplate) => (
                                        <MenuItem value={taskTemplate.id} key={taskTemplate.id}>
                                            {taskTemplate.name}
                                        </MenuItem>
                                    ))}
                                </TextField>
                            )}
                        />
                    </div>
                </DialogContent>

                <GeneralDialogActionButtons dialogType={leafTypesConfigDialog.type} isValid={isValid} />
            </form>
        </Dialog>
    );
}

export default LeafTypesConfigDialog;
