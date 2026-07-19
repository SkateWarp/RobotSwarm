import { yupResolver } from "@hookform/resolvers/yup";
import { Visibility, VisibilityOff } from "@mui/icons-material";
import {
    Alert,
    Button,
    Dialog,
    DialogActions,
    DialogContent,
    DialogTitle,
    IconButton,
    InputAdornment,
    MenuItem,
    TextField,
    Typography,
} from "@mui/material";
import { useEffect, useMemo, useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import * as yup from "yup";
import { showMessage } from "../../../store/fuse/messageSlice";
import {
    addAccount,
    closeEditAccountDialog,
    closeNewAccountDialog,
    updateAccount,
} from "./store/accountsSlice";

const emptyAccount = {
    id: "",
    firstName: "",
    lastName: "",
    email: "",
    password: "",
    role: "User",
};

const accountSchema = (editing) =>
    yup.object().shape({
        firstName: yup.string().trim().required("Ingrese el nombre."),
        lastName: yup.string().trim().required("Ingrese el apellido."),
        email: yup
            .string()
            .trim()
            .email("Ingrese un correo electrónico válido.")
            .required("Ingrese el correo electrónico."),
        role: yup.string().oneOf(["Admin", "User"]).required("Seleccione un rol."),
        password: editing
            ? yup
                  .string()
                  .test(
                      "optional-password",
                      "La contraseña debe tener al menos 8 caracteres.",
                      (value) => !value || value.length >= 8
                  )
            : yup.string().required("Ingrese una contraseña.").min(8, "Use al menos 8 caracteres."),
    });

const getSubmitError = (error) => {
    if (typeof error === "string") {
        return error;
    }
    return error?.message || "No fue posible guardar la cuenta.";
};

function GtsAccountDialog() {
    const dispatch = useDispatch();
    const accountDialog = useSelector(({ accountsApp }) => accountsApp.accounts.accountDialog);
    const editing = accountDialog.type === "edit";
    const [saving, setSaving] = useState(false);
    const [showPassword, setShowPassword] = useState(false);
    const [submitError, setSubmitError] = useState("");
    const schema = useMemo(() => accountSchema(editing), [editing]);
    const {
        control,
        formState: { errors, isValid },
        handleSubmit,
        reset,
    } = useForm({
        defaultValues: emptyAccount,
        mode: "onChange",
        resolver: yupResolver(schema),
    });

    useEffect(() => {
        if (!accountDialog.props.open) {
            return;
        }

        if (editing && accountDialog.data) {
            reset({
                id: accountDialog.data.id,
                firstName: accountDialog.data.firstName || "",
                lastName: accountDialog.data.lastName || "",
                email: accountDialog.data.email || "",
                password: "",
                role: accountDialog.data.role === "Admin" ? "Admin" : "User",
            });
        } else {
            reset(emptyAccount);
        }
        setShowPassword(false);
        setSubmitError("");
    }, [accountDialog.data, accountDialog.props.open, editing, reset]);

    const closeDialog = () => {
        if (saving) {
            return;
        }
        dispatch(editing ? closeEditAccountDialog() : closeNewAccountDialog());
    };

    const onSubmit = async (values) => {
        const account = {
            firstName: values.firstName.trim(),
            lastName: values.lastName.trim(),
            email: values.email.trim().toLocaleLowerCase(),
            role: values.role,
        };

        if (values.password) {
            account.password = values.password;
        }

        setSaving(true);
        setSubmitError("");
        try {
            if (editing) {
                await dispatch(updateAccount({ id: values.id, ...account })).unwrap();
            } else {
                await dispatch(addAccount({ ...account, password: values.password })).unwrap();
            }
            dispatch(
                showMessage({
                    message: editing ? "Cuenta actualizada." : "Cuenta creada.",
                    variant: "success",
                })
            );
            dispatch(editing ? closeEditAccountDialog() : closeNewAccountDialog());
        } catch (error) {
            setSubmitError(getSubmitError(error));
        } finally {
            setSaving(false);
        }
    };

    return (
        <Dialog
            aria-labelledby="account-dialog-title"
            fullWidth
            maxWidth="sm"
            open={accountDialog.props.open}
            onClose={closeDialog}
        >
            <form onSubmit={handleSubmit(onSubmit)} noValidate>
                <DialogTitle id="account-dialog-title">
                    {editing ? "Editar cuenta" : "Crear cuenta"}
                </DialogTitle>
                <DialogContent className="flex flex-col gap-16 pt-8">
                    <Typography color="textSecondary" variant="body2">
                        Las cuentas se administran desde este panel; el registro público está deshabilitado.
                    </Typography>

                    {submitError ? (
                        <Alert severity="error" role="alert">
                            {submitError}
                        </Alert>
                    ) : null}

                    <div className="flex flex-col sm:flex-row gap-16">
                        <Controller
                            name="firstName"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    autoComplete="given-name"
                                    disabled={saving}
                                    error={!!errors.firstName}
                                    fullWidth
                                    helperText={errors.firstName?.message}
                                    label="Nombre"
                                />
                            )}
                        />
                        <Controller
                            name="lastName"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    autoComplete="family-name"
                                    disabled={saving}
                                    error={!!errors.lastName}
                                    fullWidth
                                    helperText={errors.lastName?.message}
                                    label="Apellido"
                                />
                            )}
                        />
                    </div>

                    <Controller
                        name="email"
                        control={control}
                        render={({ field }) => (
                            <TextField
                                {...field}
                                autoComplete="email"
                                disabled={saving}
                                error={!!errors.email}
                                fullWidth
                                helperText={errors.email?.message}
                                label="Correo electrónico"
                                type="email"
                            />
                        )}
                    />

                    <Controller
                        name="password"
                        control={control}
                        render={({ field }) => (
                            <TextField
                                {...field}
                                autoComplete="new-password"
                                disabled={saving}
                                error={!!errors.password}
                                fullWidth
                                helperText={
                                    errors.password?.message ||
                                    (editing
                                        ? "Déjela vacía para conservar la contraseña actual."
                                        : "Mínimo 8 caracteres.")
                                }
                                label={editing ? "Nueva contraseña (opcional)" : "Contraseña"}
                                type={showPassword ? "text" : "password"}
                                InputProps={{
                                    endAdornment: (
                                        <InputAdornment position="end">
                                            <IconButton
                                                aria-label={
                                                    showPassword ? "Ocultar contraseña" : "Mostrar contraseña"
                                                }
                                                edge="end"
                                                onClick={() => setShowPassword((visible) => !visible)}
                                                type="button"
                                            >
                                                {showPassword ? <VisibilityOff /> : <Visibility />}
                                            </IconButton>
                                        </InputAdornment>
                                    ),
                                }}
                            />
                        )}
                    />

                    <Controller
                        name="role"
                        control={control}
                        render={({ field }) => (
                            <TextField
                                {...field}
                                disabled={saving}
                                error={!!errors.role}
                                fullWidth
                                helperText={errors.role?.message}
                                label="Rol"
                                select
                            >
                                <MenuItem value="User">Usuario</MenuItem>
                                <MenuItem value="Admin">Administrador</MenuItem>
                            </TextField>
                        )}
                    />
                </DialogContent>
                <DialogActions className="px-24 pb-24">
                    <Button disabled={saving} onClick={closeDialog}>
                        Cancelar
                    </Button>
                    <Button color="primary" disabled={saving || !isValid} type="submit" variant="contained">
                        {saving ? "Guardando…" : "Guardar"}
                    </Button>
                </DialogActions>
            </form>
        </Dialog>
    );
}

export default GtsAccountDialog;
