import { yupResolver } from "@hookform/resolvers/yup";
import AppBar from "@mui/material/AppBar";
import Dialog from "@mui/material/Dialog";
import DialogContent from "@mui/material/DialogContent";
import Icon from "@mui/material/Icon";
import TextField from "@mui/material/TextField";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import { useCallback, useEffect } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import * as yup from "yup";
import MenuItem from "@mui/material/MenuItem";
import {
    addNewTaskType,
    closeEditTaskConfigDialog,
    closeNewTaskConfigDialog,
    updateTaskType,
} from "./store/realtimeConfigSlice";
import * as Actions from "../../../../store/fuse/messageSlice";
import GeneralDialogActionButtons from "../../../../shared-components/GeneralDialogActionButtons";

const taskCategories = [
    {
        id: 0,
        description: "Ninguno",
    },
    {
        id: 1,
        description: "Transporte",
    },
    {
        id: 2,
        description: "Sigue al líder",
    },
    {
        id: 3,
        description: "Formación",
    },
];

const defaultValues = {
    name: "",
    description: "",
    taskType: 0,
};

const schema = yup.object().shape({
    name: yup.string().required("Debe ingresar un nombre").min(3, "Debe ser mayor de 2 caracteres."),
    taskType: yup.number().required(),
    // description:yup.string().required("Debe ingresar una descripción").min(3, "Debe ser mayor de 2 caracteres")
});

function RealtimeConfigDialog() {
    const dispatch = useDispatch();
    const taskConfigDialog = useSelector(({ realtimeConfigApp }) => realtimeConfigApp.config.configDialog);

    const {
        control,
        reset,
        handleSubmit,
        formState: { errors, isValid },
    } = useForm({
        mode: "onChange",
        defaultValues,
        resolver: yupResolver(schema),
    });

    const initDialog = useCallback(() => {
        if (taskConfigDialog.type === "edit" && taskConfigDialog.data) {
            reset({ ...taskConfigDialog.data, taskType: taskConfigDialog.data.taskType });
        }

        if (taskConfigDialog.type === "new") {
            reset(defaultValues);
        }
    }, [taskConfigDialog.data, taskConfigDialog.type, reset]);

    useEffect(() => {
        if (taskConfigDialog.props.open) {
            initDialog();
        }
    }, [taskConfigDialog.props.open, initDialog]);

    function closeComposeDialog() {
        return taskConfigDialog.type === "edit"
            ? dispatch(closeEditTaskConfigDialog())
            : dispatch(closeNewTaskConfigDialog());
    }

    function onSubmit(data) {
        if (taskConfigDialog.type === "new") {
            dispatch(Actions.showMessage({ message: "Creando..." }));
            dispatch(addNewTaskType(data));
        } else {
            dispatch(Actions.showMessage({ message: "Actualizando..." }));
            dispatch(updateTaskType({ ...taskConfigDialog.data, ...data }));
        }

        closeComposeDialog();
    }

    return (
        <Dialog
            classes={{
                paper: "m-24",
            }}
            {...taskConfigDialog.props}
            onClose={closeComposeDialog}
            fullWidth
            maxWidth="xs"
        >
            <AppBar position="static" elevation={0}>
                <Toolbar className="flex w-full">
                    <Typography variant="subtitle1" color="inherit">
                        {taskConfigDialog.type === "new" ? "Nuevo" : "Editar"}
                    </Typography>
                </Toolbar>
                <div className="flex flex-col items-center justify-center pb-24">
                    {taskConfigDialog.type === "edit" && (
                        <Typography variant="h6" color="inherit" className="pt-8">
                            {/* {productCode} */}
                        </Typography>
                    )}
                </div>
            </AppBar>
            <form noValidate onSubmit={handleSubmit(onSubmit)} className="flex flex-col md:overflow-hidden">
                <DialogContent classes={{ root: "p-24" }}>
                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">account_circle</Icon>
                        </div>
                        <Controller
                            control={control}
                            name="name"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Nombre"
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
                            name="description"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Descripción"
                                    variant="outlined"
                                    type="text"
                                    // error={!!errors.description}
                                    // helperText={errors?.description?.message}
                                    // required
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
                            name="taskType"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    autoFocus
                                    select
                                    margin="dense"
                                    label="Categoría"
                                    className="w-full"
                                    required
                                >
                                    {taskCategories.map((taskCategory) => (
                                        <MenuItem value={taskCategory.id} key={taskCategory.id}>
                                            {taskCategory.description}
                                        </MenuItem>
                                    ))}
                                </TextField>
                            )}
                        />
                    </div>
                </DialogContent>

                <GeneralDialogActionButtons dialogType={taskConfigDialog.type} isValid={isValid} />
            </form>
        </Dialog>
    );
}

export default RealtimeConfigDialog;
