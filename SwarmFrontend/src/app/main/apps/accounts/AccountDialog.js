import { useCallback, useEffect, useState } from "react";
import { yupResolver } from "@hookform/resolvers/yup";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import * as yup from "yup";
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
    Tab,
    Tabs,
    TextField,
    Toolbar,
    Typography,
} from "@mui/material";
import {
    closeEditAccountDialog,
    closeNewAccountDialog,
    createNotificationAccountRequest,
    createPermissionAccountRequest,
    getAccountPermission,
    getAccounts,
    getMachinesForNotificationsByAccountIdAndAlertType,
    removeAccount,
    submitAccount,
    updateAccount,
} from "./store/accountsSlice";
import AccountMultipleSelectCheckbox from "./AccountMultipleSelectCheckbox";
import { submitOperator } from "./store/operatorSlice";
import * as Actions from "../../../store/fuse/messageSlice";
import useCheckMobileScreen from "../../../shared-components/hooks/useCheckMobileScreen";
import settingsConfig from "../../../fuse-configs/settingsConfig";

const roles = [
    {
        value: "Admin",
        label: "Administrador",
    },
    {
        value: "User",
        label: "Usuario",
    },
    {
        value: "Operator",
        label: "Operador",
    },
    {
        value: "Maintenance",
        label: "Mantenimiento",
    },
    {
        value: "WasteOperator",
        label: "Operador de Desperdicios",
    },
    {
        value: "CleanOperator",
        label: "Operador de Limpieza",
    },
];

const rolesFraga = [
    {
        value: "Admin",
        label: "Administrador",
    },
    {
        value: "User",
        label: "Usuario",
    },
    {
        value: "Sales",
        label: "Ventas",
    },
    {
        value: "Operator",
        label: "Operador",
    },
    {
        value: "Maintenance",
        label: "Mantenimiento",
    },
    {
        value: "Reception",
        label: "Recepción",
    },
    {
        value: "Supervisor",
        label: "Supervisor",
    },
];

const defaultValues = {
    id: "",
    firstName: "",
    lastName: "",
    role: "",
    email: "",
    pin: "",
    password: "",
    confirmPassword: "",
    enabled: true,
};

let schema = yup.object();

// Todo a la hora de crear usuario con notificaciones falla y ala hora de actualizar.
function AccountDialog() {
    const dispatch = useDispatch();
    const accountDialog = useSelector(({ accountsApp }) => accountsApp.accounts.accountDialog);
    const [showPin, setShowPin] = useState(false);
    const [showPassword, setShowPassword] = useState(false);
    const [tabValue, setTabValue] = useState(0);
    const [chkRecepcion, setChkRecepcion] = useState(false);
    const [chkProduccion, setChkProduccion] = useState(false);
    const [tanksVinOilSelectedOptions, setTanksVinOilSelectedOptions] = useState([]);
    const [mayoLineSelectedOptions, setMayoLineSelectedOptions] = useState([]);
    const [eggLineSelectedOptions, setEggLineSelectedOptions] = useState([]);
    const [receptionOptions, setReceptionOptions] = useState([]);
    const [fragaOptions, setFragaOptions] = useState([]);
    const [receptionSelectedOptions, setReceptionSelectedOptions] = useState([]);
    const [slitterSelectedOptions, setSlitterSelectedOptions] = useState([]);
    const [tubemillSelectedOptions, setTubemillSelectedOptions] = useState([]);
    const [notificationOptions, setNotificationOptions] = useState([]);
    const isMobile = useCheckMobileScreen();

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
    const rol = watch("role");
    const firstName = watch("firstName");
    const lastName = watch("lastName");
    const email = watch("email");
    const pin = watch("pin");
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

            // Hago esta validacion para evitar el error de cuando se cambia de un usuario-admin a operador y este usuario
            // No tiene pin
            if (accountDialog.data.pin) setValue("pin", accountDialog.data.pin);
            else setValue("pin", "");

            console.debug("account:", accountDialog.data);

            setValue("id", accountDialog.data.id);

            if (settingsConfig.layout.project === "baldom") {
                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 2).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => notificationOptions.find((x) => x.value === alertType).label
                            );
                            setTanksVinOilSelectedOptions(notifications);
                        }
                    );

                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 1).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => notificationOptions.find((x) => x.value === alertType).label
                            );
                            setEggLineSelectedOptions(notifications);
                        }
                    );

                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 3).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => notificationOptions.find((x) => x.value === alertType).label
                            );
                            setMayoLineSelectedOptions(notifications);
                        }
                    );
            } else if (settingsConfig.layout.project === "fraga") {
                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 3).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => receptionOptions.find((x) => x.value === alertType).label
                            );
                            setReceptionSelectedOptions(notifications);
                        }
                    );
                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 2).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => fragaOptions.find((x) => x.value === alertType).label
                            );
                            setTubemillSelectedOptions(notifications);
                        }
                    );
                accountDialog.data &&
                    getMachinesForNotificationsByAccountIdAndAlertType(accountDialog.data.id, 1).then(
                        (data) => {
                            const notifications = data.map(
                                ({ alertType }) => fragaOptions.find((x) => x.value === alertType).label
                            );
                            setSlitterSelectedOptions(notifications);
                        }
                    );

                accountDialog.data &&
                    getAccountPermission(accountDialog.data.id).then((data) => {
                        data.forEach(function (element) {
                            if (element.appPermissions === "Recepcion") setChkRecepcion(true);
                            if (element.appPermissions === "Produccion") setChkProduccion(true);
                        });
                    });
            }
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

        setTabValue(0);
    }, [accountDialog.data, accountDialog.type, setValue, setTabValue]);

    useEffect(() => {
        setReceptionOptions([{ value: 3, label: "Reporte" }]);

        setFragaOptions([
            { value: 0, label: "Paradas" },
            { value: 1, label: "Máquina" },
            { value: 2, label: "Producción" },
            { value: 4, label: "Mantenimientos" },
            { value: 5, label: "Turnos" },
        ]);

        setNotificationOptions([
            { value: 0, label: "Paradas" },
            { value: 1, label: "Alarmas Máquina" },
            { value: 2, label: "Alarmas Producción" },
            { value: 4, label: "Mantenimientos" },
        ]);

        if (accountDialog.props.open) {
            initDialog();
        }
    }, [accountDialog.props.open, initDialog, dispatch]);

    useEffect(() => {
        setUpValidation(rol);
    }, [rol]);

    const setUpValidation = (rol) => {
        if (accountDialog.props.open) {
            if (
                rol === "Operator" ||
                (rol === "Maintenance" &&
                    (settingsConfig.layout.project === "baldom" ||
                        settingsConfig.layout.project === "task")) ||
                rol === "WasteOperator" ||
                rol === "CleanOperator"
            ) {
                schema = yup.object().shape({
                    firstName: yup
                        .string()
                        .required("Debe ingresar un nombre.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    lastName: yup
                        .string()
                        .required("Debe ingresar un apellido.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    email: yup.string().email("Debe ingresar un correo eletrónico válido."),
                    pin: yup
                        .string()
                        .required("Ingrese su pin")
                        .min(4, "El pin debe constar de 4 dígitos.")
                        .max(4, "El pin debe constar de 4 dígitos.")
                        .matches(/^[0-9]+$/, "El pin debe contener solo números.")
                        .nullable(),
                });
            } else if (rol === "Admin" || rol === "User" || rol === "Reception" || rol === "Sales") {
                schema = yup.object().shape({
                    firstName: yup
                        .string()
                        .required("Debe ingresar un nombre.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    lastName: yup
                        .string()
                        .required("Debe ingresar un apellido.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    email: yup
                        .string()
                        .email("Debe ingresar un correo eletrónico válido.")
                        .required("Debe ingresar un correo electrónico."),
                    password: yup
                        .string()
                        .required("Por favor, ingrese su contraseña.")
                        .min(8, "El mínimo de caracteres debe ser 8.")
                        .max(16, "El máximo de caracteres debe ser 16.")
                        .matches(
                            /^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$/,
                            "Debe tener al menos 1 letra, 1 número y 1 símbolo !@#$%^&*"
                        ),
                    confirmPassword: yup
                        .string()
                        .required("Debe confirmar su contraseña.")
                        .oneOf([yup.ref("password"), null], "Las contraseñas no son iguales."),
                });
            } else if (
                rol === "Supervisor" ||
                (rol === "Maintenance" && settingsConfig.layout.project === "fraga")
            ) {
                schema = yup.object().shape({
                    firstName: yup
                        .string()
                        .required("Debe ingresar un nombre.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    lastName: yup
                        .string()
                        .required("Debe ingresar un apellido.")
                        .min(3, "Debe ser mayor de 2 caracteres."),
                    email: yup
                        .string()
                        .email("Debe ingresar un correo eletrónico válido.")
                        .required("Debe ingresar un correo electrónico."),
                    password: yup
                        .string()
                        .required("Por favor, ingrese su contraseña.")
                        .min(8, "El mínimo de caracteres debe ser 8.")
                        .max(16, "El máximo de caracteres debe ser 16.")
                        .matches(
                            /^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$/,
                            "Debe tener al menos 1 letra, 1 número y 1 símbolo !@#$%^&*"
                        ),
                    confirmPassword: yup
                        .string()
                        .required("Debe confirmar su contraseña.")
                        .oneOf([yup.ref("password"), null], "Las contraseñas no son iguales."),
                    pin: yup
                        .string()
                        .required("Ingrese su pin")
                        .min(4, "El pin debe constar de 4 dígitos.")
                        .max(4, "El pin debe constar de 4 dígitos.")
                        .matches(/^[0-9]+$/, "El pin debe contener solo números.")
                        .nullable(),
                });
            }
        }
    };

    function closeComposeDialog() {
        resetNotifications();

        return accountDialog.type === "edit"
            ? dispatch(closeEditAccountDialog())
            : dispatch(closeNewAccountDialog());
    }

    const getNotificationValueByLabel = (notifications) => {
        const completeNotificationObject = [];

        notifications.map((actualValue) => {
            const newData = notificationOptions.find((data) => data.label === actualValue);

            completeNotificationObject.push(newData);
        });

        return completeNotificationObject;
    };

    const getFragaNotificationValueByLabel = (notifications, machine) => {
        const completeNotificationObject = [];

        if (machine === "tubemill" || machine === "slitter") {
            notifications.map((actualValue) => {
                const newData = fragaOptions.find((data) => data.label === actualValue);
                completeNotificationObject.push(newData);
            });
        } else {
            notifications.map((actualValue) => {
                const newData = receptionOptions.find((data) => data.label === actualValue);
                completeNotificationObject.push(newData);
            });
        }

        return completeNotificationObject;
    };

    function handleChangeProduccion() {
        setChkProduccion(!chkProduccion);
    }

    function handleChangeRecepcion() {
        setChkRecepcion(!chkRecepcion);
    }

    function handleSubmitNotification(accountId) {
        const completeEggLineSelectedOptions = getNotificationValueByLabel(eggLineSelectedOptions);
        const completeTanksSelectedOptions = getNotificationValueByLabel(tanksVinOilSelectedOptions);
        const completeMayoSelectedOptions = getNotificationValueByLabel(mayoLineSelectedOptions);

        const completeReceptionSelectedOptions = getFragaNotificationValueByLabel(
            receptionSelectedOptions,
            "reception"
        );
        const completeSlitterSelectedOptions = getFragaNotificationValueByLabel(
            slitterSelectedOptions,
            "slitter"
        );
        const completeTubemillSelectedOptions = getFragaNotificationValueByLabel(
            tubemillSelectedOptions,
            "tubemill"
        );

        let model;

        if (settingsConfig.layout.project === "fraga") {
            model = prepareFragaNotificationRequest(accountId, 2, completeTubemillSelectedOptions);
            model.push(...prepareFragaNotificationRequest(accountId, 1, completeSlitterSelectedOptions));
            model.push(...prepareFragaNotificationRequest(accountId, 3, completeReceptionSelectedOptions));

            if (model.length === 0) {
                model = [{ accountId, machineId: 0, alertType: 0 }];
            }
        } else {
            model = prepareNotificationRequest(accountId, 2, completeTanksSelectedOptions);
            model.push(...prepareNotificationRequest(accountId, 1, completeEggLineSelectedOptions));
            model.push(...prepareNotificationRequest(accountId, 3, completeMayoSelectedOptions));

            if (model.length === 0) {
                model = [{ accountId, lineProductionId: 0, alertType: 0 }];
            }
        }

        // if (model.length === 0) {
        //     model = [{accountId, lineProductionId: 0, alertType: 0}];
        // }

        createNotificationAccountRequest(model).then(() => {});

        closeComposeDialog();
    }

    function handleSubmitPermissions(accountId) {
        let submitPermissions;

        if (!chkProduccion && !chkRecepcion) {
            submitPermissions = [{ accountId, appPermissions: 0, create: false }];
        }
        if (chkRecepcion) {
            submitPermissions = [{ accountId, appPermissions: 0, create: true }];
        }
        if (chkProduccion) {
            submitPermissions = [{ accountId, appPermissions: 1, create: true }];
        }

        if (chkProduccion && chkRecepcion) {
            submitPermissions = [
                { accountId, appPermissions: 0, create: true },
                { accountId, appPermissions: 1, create: true },
            ];
        }

        createPermissionAccountRequest(submitPermissions).then(() => {
            /* props.setTabValue(2); */
        });
        closeComposeDialog();
    }

    function prepareNotificationRequest(accountId, lineProductionId, alertTypes) {
        return alertTypes.map((alertType) => ({
            accountId,
            lineProductionId,
            alertType: alertType.value,
        }));
    }

    function prepareFragaNotificationRequest(accountId, machineId, alertTypes) {
        return alertTypes.map((alertType) => ({
            accountId,
            machineId,
            alertType: alertType.value,
        }));
    }

    function resetNotifications() {
        setReceptionSelectedOptions([]);
        setSlitterSelectedOptions([]);
        setTubemillSelectedOptions([]);
        setChkProduccion(false);
        setChkRecepcion(false);

        setTanksVinOilSelectedOptions([]);
        setMayoLineSelectedOptions([]);
        setEggLineSelectedOptions([]);
    }

    async function onSubmit(data) {
        if (accountDialog.type === "new") {
            if (settingsConfig.layout.project === "baldom" || settingsConfig.layout.project === "task") {
                if (
                    rol === "Operator" ||
                    rol === "Maintenance" ||
                    rol === "WasteOperator" ||
                    rol === "CleanOperator"
                ) {
                    dispatch(Actions.showMessage({ message: "Creando nuevo operador..." }));
                    const res = await dispatch(submitOperator(data));
                    if (res.payload.id) {
                        handleSubmitNotification(res.payload.id);
                        closeComposeDialog();
                        setShowPin(false);
                        dispatch(getAccounts());
                    }
                } else {
                    dispatch(Actions.showMessage({ message: "Creando nuevo usuario..." }));
                    const res = await dispatch(submitAccount(data));

                    if (res.payload.id) {
                        handleSubmitNotification(res.payload.id);
                        closeComposeDialog();
                        dispatch(getAccounts());
                    }
                }
            } else if (rol === "Operator" || rol === "Maintenance" || rol === "Supervisor") {
                dispatch(Actions.showMessage({ message: "Creando..." }));
                const res = await dispatch(submitOperator(data));
                if (res.payload.id) {
                    handleSubmitNotification(res.payload.id);
                    handleSubmitPermissions(res.payload.id);
                    closeComposeDialog();
                    setShowPin(false);
                    dispatch(getAccounts());
                }
            } else {
                dispatch(Actions.showMessage({ message: "Creando nuevo usuario..." }));
                const res = await dispatch(submitAccount(data));

                if (res.payload.id) {
                    handleSubmitNotification(res.payload.id);
                    handleSubmitPermissions(res.payload.id);
                    closeComposeDialog();
                    dispatch(getAccounts());
                }
            }
        } else if (settingsConfig.layout.project === "fraga") {
            dispatch(updateAccount({ id: accountDialog.data.id, ...data }));
            handleSubmitNotification(accountDialog.data.id);
            handleSubmitPermissions(accountDialog.data.id);
            dispatch(getAccounts());
            closeComposeDialog();
        } else {
            dispatch(updateAccount({ id: accountDialog.data.id, ...data }));
            handleSubmitNotification(accountDialog.data.id);
            dispatch(getAccounts());
            closeComposeDialog();
        }
        resetNotifications();
    }

    function handleRemove() {
        dispatch(removeAccount({ idForDelete: id }));
        closeComposeDialog();
    }

    function handleChangeTab(event, value) {
        setTabValue(value);
    }

    function handleClickShowPassword() {
        setShowPassword(!showPassword);
    }

    function handleClickShowPin() {
        setShowPin(!showPin);
    }

    function checkForValidPassword(password) {
        return (
            /[a-z]/.test(password) &&
            /[0-9]/.test(password) &&
            /(?=.*[!@#$%^&*])/.test(password) &&
            password.length >= 8 &&
            password.length <= 16
        );
    }

    const checkForValidEmail = (email) => {
        return /^[a-zA-Z0-9.!#$%&'*+/=?^_`{|}~-]+@[a-zA-Z0-9-]+(?:\.[a-zA-Z0-9-]+)*/.test(email);
    };

    function canBeSubmitted() {
        if (
            rol === "Operator" ||
            (rol === "Maintenance" &&
                (settingsConfig.layout.project === "baldom" || settingsConfig.layout.project === "task")) ||
            rol === "WasteOperator" ||
            rol === "CleanOperator"
        ) {
            return firstName.length > 2 && lastName.length > 2 && pin.length > 3 && pin.length <= 4;
        }
        if (
            (rol === "Admin" || rol === "User" || rol === "Reception" || rol === "Sales") &&
            accountDialog.type === "edit" &&
            accountDialog.data
        ) {
            return firstName.length > 2 && lastName.length > 2 && checkForValidEmail(email);
        }
        if (rol === "Admin" || rol === "User" || rol === "Reception" || rol === "Sales") {
            return (
                firstName.length > 2 &&
                lastName.length > 2 &&
                checkForValidEmail(email) &&
                checkForValidPassword(password) &&
                confirmPassword === password
            );
        }
        if (
            (rol === "Supervisor" || (rol === "Maintenance" && settingsConfig.layout.project === "fraga")) &&
            accountDialog.type === "edit" &&
            accountDialog.data
        ) {
            return (
                firstName.length > 2 &&
                lastName.length > 2 &&
                checkForValidEmail(email) &&
                pin.length > 3 &&
                pin.length <= 4
            );
        }
        if (
            (rol === "Supervisor" || (rol === "Maintenance" && settingsConfig.layout.project === "fraga")) &&
            accountDialog.type === "new"
        ) {
            return (
                firstName.length > 2 &&
                lastName.length > 2 &&
                checkForValidEmail(email) &&
                pin.length > 3 &&
                pin.length <= 4 &&
                checkForValidPassword(password) &&
                confirmPassword === password
            );
        }
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

            <Tabs
                value={tabValue}
                onChange={handleChangeTab}
                indicatorColor="primary"
                textColor="primary"
                variant="scrollable"
                scrollButtons="auto"
                classes={{ root: "w-full h-64" }}
            >
                <Tab className="h-64 normal-case" label="Datos" />
                {settingsConfig.layout.project !== "task" && (
                    <Tab className="h-64 normal-case" label="Notificaciones" />
                )}
                {settingsConfig.layout.project === "fraga" && (
                    <Tab className="h-64 normal-case" label="Permisos" />
                )}
            </Tabs>

            <form noValidate onSubmit={handleSubmit(onSubmit)} className="flex flex-col md:overflow-hidden">
                <DialogContent classes={{ root: "p-24" }}>
                    {tabValue === 0 && (
                        <>
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
                                            component=""
                                            sx={{
                                                "& .MuiTextField-root": {
                                                    width: !isMobile ? "40.5ch" : "29.5ch",
                                                },
                                            }}
                                            noValidate
                                            autoComplete="off"
                                        >
                                            {(settingsConfig.layout.project === "baldom" ||
                                                settingsConfig.layout.project === "task") && (
                                                <TextField
                                                    {...field}
                                                    className="mb-24"
                                                    select
                                                    label="Rol"
                                                    required
                                                >
                                                    {roles.map((option) => (
                                                        <MenuItem key={option.value} value={option.value}>
                                                            {option.label}
                                                        </MenuItem>
                                                    ))}
                                                </TextField>
                                            )}
                                            {settingsConfig.layout.project === "fraga" && (
                                                <TextField
                                                    {...field}
                                                    className="mb-24"
                                                    select
                                                    label="Rol"
                                                    required
                                                >
                                                    {rolesFraga.map((option) => (
                                                        <MenuItem key={option.value} value={option.value}>
                                                            {option.label}
                                                        </MenuItem>
                                                    ))}
                                                </TextField>
                                            )}
                                        </Box>
                                    )}
                                />
                            </div>

                            {(rol === "Operator" ||
                                rol === "Maintenance" ||
                                rol === "WasteOperator" ||
                                rol === "CleanOperator" ||
                                rol === "Supervisor") && (
                                <div className="flex">
                                    <div className="min-w-48 pt-20">
                                        <Icon color="action">apps</Icon>
                                    </div>

                                    <Controller
                                        name="pin"
                                        control={control}
                                        render={({ field }) => (
                                            <TextField
                                                {...field}
                                                className="mb-24"
                                                type={showPin ? "text" : "password"}
                                                label="Pin"
                                                id="pin"
                                                error={!!errors.pin}
                                                helperText={errors?.pin?.message}
                                                InputProps={{
                                                    endAdornment: (
                                                        <InputAdornment position="end">
                                                            <IconButton
                                                                aria-label="ver/ocultar pin"
                                                                onClick={handleClickShowPin}
                                                                edge="end"
                                                            >
                                                                {showPin ? <Visibility /> : <VisibilityOff />}
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
                            )}

                            {rol === "Admin" || rol === "User" || rol === "Sales" ? (
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
                            ) : (
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
                                            />
                                        )}
                                    />
                                </div>
                            )}

                            {(rol === "Admin" ||
                                rol === "User" ||
                                rol === "Sales" ||
                                (rol === "Maintenance" && settingsConfig.layout.project === "fraga") ||
                                rol === "Reception" ||
                                rol === "Supervisor") && (
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
                                                                {showPassword ? (
                                                                    <Visibility />
                                                                ) : (
                                                                    <VisibilityOff />
                                                                )}
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
                            )}

                            {(rol === "Admin" ||
                                rol === "User" ||
                                rol === "Sales" ||
                                (rol === "Maintenance" && settingsConfig.layout.project === "fraga") ||
                                rol === "Supervisor" ||
                                rol === "Reception") && (
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
                                                                {showPassword ? (
                                                                    <Visibility />
                                                                ) : (
                                                                    <VisibilityOff />
                                                                )}
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
                            )}

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
                        </>
                    )}

                    {tabValue === 1 && settingsConfig.layout.project === "baldom" && (
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
                    )}

                    {tabValue === 1 && settingsConfig.layout.project === "fraga" && (
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
                    )}

                    {settingsConfig.layout.project === "fraga" &&
                        tabValue === 2 &&
                        settingsConfig.layout.project === "fraga" && (
                            <div className="flex flex-col w-full max-w-sm">
                                <div className="p-10">Permisos para aplicación móvil:</div>
                                <div className="pl-10">
                                    <Controller
                                        name="chkRecepcion"
                                        control={control}
                                        render={({ field: {} }) => (
                                            <FormControlLabel
                                                label="Recepción"
                                                control={
                                                    <Checkbox
                                                        checked={chkRecepcion}
                                                        onChange={handleChangeRecepcion}
                                                    />
                                                }
                                            />
                                        )}
                                    />
                                </div>

                                <div className="pl-10">
                                    <Controller
                                        name="chkProduccion"
                                        control={control}
                                        render={({ field: {} }) => (
                                            <FormControlLabel
                                                label="Producción"
                                                control={
                                                    <Checkbox
                                                        checked={chkProduccion}
                                                        onChange={handleChangeProduccion}
                                                    />
                                                }
                                            />
                                        )}
                                    />
                                </div>
                            </div>
                        )}
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

export default AccountDialog;
