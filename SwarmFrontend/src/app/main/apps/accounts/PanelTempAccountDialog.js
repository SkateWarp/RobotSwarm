import { useCallback, useEffect, useState } from "react";
import { yupResolver } from "@hookform/resolvers/yup";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import { Visibility, VisibilityOff } from "@mui/icons-material";
import {
    AppBar,
    Box,
    Button,
    Checkbox,
    Dialog,
    DialogActions,
    DialogContent,
    FormControlLabel,
    Icon,
    IconButton,
    InputAdornment,
    MenuItem,
    TextField,
    Toolbar,
    Typography,
} from "@mui/material";
import * as yup from "yup";
import {
    closeEditAccountDialog,
    closeNewAccountDialog,
    getAccountsByCompany,
    removeAccount,
    submitAccount,
    updateAccountPanelTemp,
} from "./store/accountsSlice";
import * as Actions from "../../../store/fuse/messageSlice";
import useCheckMobileScreen from "../../../shared-components/hooks/useCheckMobileScreen";
import useActualCompanyId from "../../../shared-components/hooks/useActualCompanyId";

const roles = [
    {
        value: "Admin",
        label: "Administrador",
    },
    {
        value: "User",
        label: "Usuario",
    },
];

const defaultValues = {
    id: "",
    firstName: "",
    lastName: "",
    role: "",
    email: "",
    password: "",
    confirmPassword: "",
    enabled: true,
};

const schema = yup.object().shape({
    firstName: yup.string().required("Debe ingresar un nombre.").min(3, "Debe ser mayor de 2 caracteres."),
    lastName: yup.string().required("Debe ingresar un apellido.").min(3, "Debe ser mayor de 2 caracteres."),
    email: yup
        .string()
        .email("Debe ingresar un correo eletrónico válido.")
        .required("Debe ingresar un correo electrónico."),
    password: yup
        .mixed()
        .test("is_empty", "", (value) => {
            if (typeof value === "string" || typeof value === undefined) {
                const passwordLength = value.length;
                if (passwordLength === 0) {
                    return true;
                }
                return true;
            }

            return false;
        })
        .test("less_than", "El mínimo de caracteres debe ser 8.", (value) => {
            if (typeof value === "string") {
                const passwordLength = value.length;
                if (passwordLength === 0) {
                    return true;
                }
                if (passwordLength < 8) {
                    return false;
                }
            }

            return true;
        })
        .test("more_than", "El máximo de caracteres debe ser 16.", (value) => {
            if (typeof value === "string") {
                const passwordLength = value.length;
                if (passwordLength === 0) {
                    return true;
                }
                if (passwordLength > 16) {
                    return false;
                }
            }

            return true;
        })
        .test("regx", "Debe tener al menos 1 letra, 1 número y 1 símbolo !@#$%^&*", (value) => {
            if (typeof value === "string") {
                const passwordLength = value.length;

                if (passwordLength === 0) {
                    return true;
                }
                return value.match(/^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$/);
            }

            return false;
        }),
    confirmPassword: yup.string().when("password", {
        is: (value) => value.length,
        then: yup
            .string()
            .required("Debe confirmar su contraseña.")
            .oneOf([yup.ref("password"), null], "Las contraseñas no son iguales."),
    }),
});

function PanelTempAccountDialog() {
    const dispatch = useDispatch();
    const companyId = useActualCompanyId();
    const isMobile = useCheckMobileScreen();
    const [showPassword, setShowPassword] = useState(false);
    const accountDialog = useSelector(({ accountsApp }) => accountsApp.accounts.accountDialog);

    const {
        control,
        watch,
        reset,
        handleSubmit,
        formState: { errors },
        setValue,
    } = useForm({
        mode: "all",
        defaultValues,
        resolver: yupResolver(schema),
    });

    const id = watch("id");
    const email = watch("email");
    const firstName = watch("firstName");
    const lastName = watch("lastName");
    const password = watch("password");
    const confirmPassword = watch("confirmPassword");

    /**
     * Initialize Dialog with Data
     */
    const initDialog = useCallback(() => {
        /**
         * Dialog type: 'edit'
         */
        if (accountDialog.type === "edit" && accountDialog.data) {
            // Le envio a los campos del formulario, los datos enviados del Dialog
            reset(accountDialog.data);

            // Password y confirmPassword llegan undefined por defecto a la hora de editar, hago esto para evitar el warning de consola
            setValue("password", "");
            setValue("confirmPassword", "");
            setValue("id", accountDialog.data.id);
        }
        /**
         * Dialog type: 'new'
         */
        if (accountDialog.type === "new") {
            reset(defaultValues);

            // Se ha agregado este setValue aqui para activar los mensajes de validaciones de yup de inmediato a
            // la hora de crear una cuenta u operador
            setValue("role", "User");
        }
    }, [accountDialog.type, accountDialog.data, reset, setValue]);

    useEffect(() => {
        if (accountDialog.props.open) {
            initDialog();
        }
    }, [accountDialog.props.open, initDialog, dispatch]);

    function closeComposeDialog() {
        return accountDialog.type === "edit"
            ? dispatch(closeEditAccountDialog())
            : dispatch(closeNewAccountDialog());
    }

    async function onSubmit(data) {
        if (accountDialog.type === "new") {
            data.companyId = companyId;
            dispatch(Actions.showMessage({ message: "Creando nuevo usuario..." }));
            const res = await dispatch(submitAccount(data));

            // @ts-ignore
            if (res.payload.id) {
                closeComposeDialog();
                dispatch(getAccountsByCompany({ companyId }));
            }
        } else {
            dispatch(updateAccountPanelTemp({ id: accountDialog.data.id, ...data }));
            dispatch(getAccountsByCompany({ companyId }));

            closeComposeDialog();
        }
    }

    function handleRemove() {
        dispatch(removeAccount({ idForDelete: id }));
        closeComposeDialog();
    }

    function handleClickShowPassword() {
        setShowPassword(!showPassword);
    }

    /**
     * @param {string} passwordToValidate
     */
    function checkForValidPassword(passwordToValidate) {
        return (
            /[a-z]/.test(passwordToValidate) &&
            /[0-9]/.test(passwordToValidate) &&
            /(?=.*[!@#$%^&*])/.test(passwordToValidate) &&
            passwordToValidate.length >= 8 &&
            passwordToValidate.length <= 16
        );
    }

    const checkForValidEmail = (/** @type {string} */ emailToValidate) => {
        return /^[a-zA-Z0-9.!#$%&'*+/=?^_`{|}~-]+@[a-zA-Z0-9-]+(?:\.[a-zA-Z0-9-]+)*/.test(emailToValidate);
    };

    function canBeSubmitted() {
        if (accountDialog.type === "edit" && accountDialog.data) {
            return firstName.length > 2 && lastName.length > 2 && checkForValidEmail(email);
        }
        return (
            firstName.length > 2 &&
            lastName.length > 2 &&
            checkForValidEmail(email) &&
            (password.length === 0 || (checkForValidPassword(password) && confirmPassword === password))
        );
    }

    return (
        <Dialog
            classes={{
                paper: "m-24",
            }}
            {...accountDialog.props}
            onClose={closeComposeDialog}
            fullWidth
            maxWidth="xs"
        >
            <AppBar position="static" elevation={0}>
                <Toolbar className="flex w-full">
                    <Typography variant="subtitle1" color="inherit">
                        {accountDialog.type === "new" ? "Nuevo Usuario" : "Editar Usuario"}
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
                            name="firstName"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Nombre"
                                    error={!!errors.firstName}
                                    helperText={errors?.firstName?.message}
                                    variant="outlined"
                                    required
                                    fullWidth
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">account_circle</Icon>
                        </div>

                        <Controller
                            control={control}
                            name="lastName"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Apellido"
                                    error={!!errors.lastName}
                                    helperText={errors?.lastName?.message}
                                    variant="outlined"
                                    required
                                    fullWidth
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">security</Icon>
                        </div>
                        <Controller
                            control={control}
                            name="role"
                            render={({ field }) => (
                                <Box
                                    sx={{
                                        "& .MuiTextField-root": {
                                            width: !isMobile ? "40.5ch" : "29.5ch",
                                        },
                                    }}
                                    noValidate
                                    autoComplete="off"
                                >
                                    <TextField {...field} className="mb-24" select label="Rol" required>
                                        {roles.map((option) => (
                                            <MenuItem key={option.value} value={option.value}>
                                                {option.label}
                                            </MenuItem>
                                        ))}
                                    </TextField>
                                </Box>
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">email</Icon>
                        </div>

                        <Controller
                            control={control}
                            name="email"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    label="Correo Eletrónico"
                                    error={!!errors.email}
                                    helperText={errors?.email?.message}
                                    variant="outlined"
                                    fullWidth
                                    required
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">lock</Icon>
                        </div>

                        <Controller
                            name="password"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    type={showPassword ? "text" : "password"}
                                    label="Contraseña"
                                    error={!!errors.password}
                                    helperText={errors?.password?.message}
                                    InputProps={{
                                        endAdornment: (
                                            <InputAdornment position="end">
                                                <IconButton
                                                    aria-label="ver/ocultar password"
                                                    onClick={handleClickShowPassword}
                                                    edge="end"
                                                >
                                                    {showPassword ? <Visibility /> : <VisibilityOff />}
                                                </IconButton>
                                            </InputAdornment>
                                        ),
                                    }}
                                    variant="outlined"
                                    fullWidth
                                    required
                                />
                            )}
                        />
                    </div>

                    <div className="flex">
                        <div className="min-w-48 pt-20">
                            <Icon color="action">lock</Icon>
                        </div>
                        <Controller
                            name="confirmPassword"
                            control={control}
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-24"
                                    type={showPassword ? "text" : "password"}
                                    label="Confirmar Contraseña"
                                    id="confirmPassword"
                                    error={!!errors.confirmPassword}
                                    helperText={errors?.confirmPassword?.message}
                                    InputProps={{
                                        endAdornment: (
                                            <InputAdornment position="end">
                                                <IconButton
                                                    aria-label="ver/ocultar password"
                                                    onClick={handleClickShowPassword}
                                                    edge="end"
                                                >
                                                    {showPassword ? <Visibility /> : <VisibilityOff />}
                                                </IconButton>
                                            </InputAdornment>
                                        ),
                                    }}
                                    variant="outlined"
                                    fullWidth
                                    required
                                />
                            )}
                        />
                    </div>

                    <div className="flex mx-48">
                        {accountDialog.type === "edit" && (
                            <Controller
                                name="enabled"
                                control={control}
                                render={({ field: { value, onChange } }) => (
                                    <FormControlLabel
                                        label="Habilitar"
                                        control={<Checkbox checked={value} onChange={onChange} />}
                                    />
                                )}
                            />
                        )}
                    </div>

                    {/* {tabValue === 1 && settingsConfig.layout.project === "baldom" && (
                        <div className="flex flex-col w-full max-w-sm">
                            <div className="p-10">Seleccione las notificaciones deseadas:</div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={notificationOptions}
                                    selectedValue={eggLineSelectedOptions}
                                    setValue={setEggLineSelectedOptions}
                                    notificationType="Línea de Huevos"
                                />
                            </div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={notificationOptions}
                                    selectedValue={tanksVinOilSelectedOptions}
                                    setValue={setTanksVinOilSelectedOptions}
                                    notificationType="Tanques de Aceite y Vinagre"
                                />
                            </div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={notificationOptions}
                                    selectedValue={mayoLineSelectedOptions}
                                    setValue={setMayoLineSelectedOptions}
                                    notificationType="Línea de Mayonesa"
                                />
                            </div>
                        </div>
                    )} */}

                    {/* {tabValue === 1 && settingsConfig.layout.project === "fraga" && (
                        <div className="flex flex-col w-full max-w-sm">
                            <div className="p-10">Seleccione las notificaciones deseadas:</div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={receptionOptions}
                                    selectedValue={receptionSelectedOptions}
                                    setValue={setReceptionSelectedOptions}
                                    notificationType="Recepción"
                                />
                            </div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={fragaOptions}
                                    selectedValue={slitterSelectedOptions}
                                    setValue={setSlitterSelectedOptions}
                                    notificationType="Slitter"
                                />
                            </div>
                            <div>
                                <AccountMultipleSelectCheckbox
                                    dataList={fragaOptions}
                                    selectedValue={tubemillSelectedOptions}
                                    setValue={setTubemillSelectedOptions}
                                    notificationType="Tube Mill"
                                />
                            </div>
                        </div>
                    )} */}
                </DialogContent>

                {accountDialog.type === "new" ? (
                    <DialogActions className="justify-between p-4 pb-16">
                        <div className="px-16">
                            <Button
                                variant="contained"
                                color="secondary"
                                type="submit"
                                disabled={!canBeSubmitted()}
                            >
                                CREAR
                            </Button>
                        </div>
                    </DialogActions>
                ) : (
                    <DialogActions className="justify-between p-4 pb-16">
                        <div className="px-16">
                            <Button
                                variant="contained"
                                color="secondary"
                                type="submit"
                                disabled={!canBeSubmitted()}
                            >
                                GUARDAR
                            </Button>
                        </div>
                        <IconButton onClick={handleRemove} size="large">
                            <Icon>delete</Icon>
                        </IconButton>
                    </DialogActions>
                )}
            </form>
        </Dialog>
    );
}

export default PanelTempAccountDialog;
