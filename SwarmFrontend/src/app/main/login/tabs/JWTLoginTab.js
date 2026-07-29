import { yupResolver } from "@hookform/resolvers/yup";
import TextField from "@mui/material/TextField";
import Alert from "@mui/material/Alert";
import Button from "@mui/material/Button";
import CircularProgress from "@mui/material/CircularProgress";
import Icon from "@mui/material/Icon";
import IconButton from "@mui/material/IconButton";
import InputAdornment from "@mui/material/InputAdornment";
import { useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import { clearLoginError, submitLogin } from "app/auth/store/loginSlice";
import * as yup from "yup";
import _ from "@lodash";

/**
 * Form Validation Schema
 */
const schema = yup.object().shape({
    email: yup
        .string()
        .email("El correo electrónico debe ser válido.")
        .required("El correo electrónico debe ser válido."),
    password: yup.string().required("Favor de introducir su contraseña."),
    // eslint-disable-next-line
    //.min(6, 'El mínimo de caracteres debe ser 6'),
});

const defaultValues = {
    email: "",
    password: "",
};

function getErrorMessage(error) {
    if (!error) {
        return "";
    }

    if (typeof error === "string") {
        return error;
    }

    const validationMessage = Object.values(error.errors || {}).flat()[0];
    return error.message || error.detail || validationMessage || "No fue posible iniciar sesión.";
}

function JWTLoginTab() {
    const dispatch = useDispatch();
    const login = useSelector(({ auth }) => auth.login);
    const { control, formState, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues,
        resolver: yupResolver(schema),
    });

    const { isValid, dirtyFields, errors } = formState;

    const [showPassword, setShowPassword] = useState(false);
    const loginError = getErrorMessage(login.error);

    function onSubmit(model) {
        dispatch(submitLogin(model));
    }

    return (
        <div className="w-full">
            <form
                className="flex flex-col justify-center w-full"
                onSubmit={handleSubmit(onSubmit)}
                aria-busy={login.busy}
                noValidate
            >
                {loginError ? (
                    <Alert className="mb-16" severity="error" role="alert">
                        {loginError}
                    </Alert>
                ) : null}

                <Controller
                    name="email"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            onChange={(event) => {
                                if (login.error) {
                                    dispatch(clearLoginError());
                                }
                                field.onChange(event);
                            }}
                            className="mb-16"
                            type="email"
                            error={!!errors.email}
                            helperText={errors?.email?.message}
                            label="Correo electrónico"
                            autoComplete="email"
                            disabled={login.busy}
                            InputProps={{
                                endAdornment: (
                                    <InputAdornment position="end">
                                        <Icon className="text-20" color="action">
                                            email
                                        </Icon>
                                    </InputAdornment>
                                ),
                            }}
                            variant="outlined"
                        />
                    )}
                />

                <Controller
                    name="password"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            onChange={(event) => {
                                if (login.error) {
                                    dispatch(clearLoginError());
                                }
                                field.onChange(event);
                            }}
                            className="mb-16"
                            label="Contraseña"
                            type="password"
                            error={!!errors.password}
                            helperText={errors?.password?.message}
                            variant="outlined"
                            autoComplete="current-password"
                            disabled={login.busy}
                            InputProps={{
                                className: "pr-2",
                                type: showPassword ? "text" : "password",
                                endAdornment: (
                                    <InputAdornment position="end">
                                        <IconButton
                                            type="button"
                                            onClick={() => setShowPassword(!showPassword)}
                                            aria-label={
                                                showPassword ? "Ocultar contraseña" : "Mostrar contraseña"
                                            }
                                            size="large"
                                        >
                                            <Icon className="text-20" color="action">
                                                {showPassword ? "visibility" : "visibility_off"}
                                            </Icon>
                                        </IconButton>
                                    </InputAdornment>
                                ),
                            }}
                            required
                        />
                    )}
                />

                <Button
                    type="submit"
                    variant="contained"
                    color="primary"
                    className="w-full mx-auto mt-16"
                    aria-label="Iniciar sesión"
                    disabled={login.busy || _.isEmpty(dirtyFields) || !isValid}
                    value="legacy"
                >
                    {login.busy ? (
                        <>
                            <CircularProgress className="mr-8" color="inherit" size={18} />
                            Ingresando…
                        </>
                    ) : (
                        "Iniciar sesión"
                    )}
                </Button>
            </form>
        </div>
    );
}

export default JWTLoginTab;
