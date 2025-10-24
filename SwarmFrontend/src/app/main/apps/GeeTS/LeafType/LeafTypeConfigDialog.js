import { yupResolver } from "@hookform/resolvers/yup";
import DialogContent from "@mui/material/DialogContent";
import { useCallback, useEffect, useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import * as yup from "yup";
import { AppBar, Dialog, Icon, MenuItem, TextField, Toolbar, Typography } from "@mui/material";
import * as Actions from "../../../../store/fuse/messageSlice";
import GeneralDialogActionButtons from "../../../../shared-components/GeneralDialogActionButtons";
import {
    addNewLeafType,
    closeEditLeafTypesConfigDialog,
    closeNewLeafTypesConfigDialog,
    updateLeafType,
} from "./store/leafTypeConfigSlice";

const defaultValues = {
    name: "",
    description: "",
};

const schema = yup.object().shape({
    name: yup.string().required("Debe ingresar un nombre").min(1, "Debe de tener mínimo 1 caracter."),
});

function LeafTypesConfigDialog() {
    const dispatch = useDispatch();
    const leafTypesConfigDialog = useSelector(
        ({ leafTypesConfigApp }) => leafTypesConfigApp.leafTypes.leafTypesConfigDialog
    );

    const [taskCategories, setTaskCategories] = useState([
        {
            id: 0,
            name: "Ninguno"
        }, {
            id: 1,
            name: "Transporte"
        },
        {
            id: 2,
            name: "Sigue al líder"
        },
        {
            id: 3,
            name: "Formación"
        },
    ]);

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
        if (leafTypesConfigDialog.type === "edit" && leafTypesConfigDialog.data) {
            reset({
                ...leafTypesConfigDialog.data,
                taskCategoryId: leafTypesConfigDialog.data.taskType || taskCategories[0].id,
            });
        }

        if (leafTypesConfigDialog.type === "new") {
            reset({
                ...defaultValues,
                taskCategoryId: taskCategories[0].id,
            });
        }
    }, [leafTypesConfigDialog.data, leafTypesConfigDialog.type, reset, taskCategories]);

    useEffect(() => {
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
            dispatch(addNewLeafType(data));
        } else {
            dispatch(Actions.showMessage({ message: "Actualizando..." }));
            dispatch(updateLeafType({ ...leafTypesConfigDialog.data, ...data }));
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
                            name="taskCategoryId"
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
                                            {taskCategory.name}
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
