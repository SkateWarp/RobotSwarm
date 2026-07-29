import { yupResolver } from "@hookform/resolvers/yup";
import Alert from "@mui/material/Alert";
import Button from "@mui/material/Button";
import CircularProgress from "@mui/material/CircularProgress";
import Icon from "@mui/material/Icon";
import IconButton from "@mui/material/IconButton";
import InputAdornment from "@mui/material/InputAdornment";
import TextField from "@mui/material/TextField";
import history from "@history";
import axios from "axios";
import { useState } from "react";
import { Controller, useForm } from "react-hook-form";
import * as yup from "yup";
import _ from "@lodash";
import { URL } from "../../../constants/constants";

const passwordMessage = "Usa entre 8 y 16 caracteres, con una minúscula, un número y un símbolo !@#$%^&*.";
const nameMessage = "El campo no puede superar 100 caracteres.";
const emailPattern = /^[A-Za-z0-9._%+-]+@(?:[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)+[A-Za-z]{2,63}$/;

export const registrationSchema = yup.object().shape({
    firstName: yup.string().trim().max(100, nameMessage).required("Favor de introducir tu nombre."),
    lastName: yup.string().trim().max(100, nameMessage).required("Favor de introducir tu apellido."),
    email: yup
        .string()
        .trim()
        .email("El correo electrónico debe ser válido.")
        .matches(emailPattern, "El correo electrónico debe ser válido.")
        .max(254, "El correo electrónico es demasiado largo.")
        .required("Favor de introducir tu correo electrónico."),
    password: yup
        .string()
        .required("Favor de introducir una contraseña.")
        .matches(/^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$/, passwordMessage),
});

const defaultValues = {
    firstName: "",
    lastName: "",
    email: "",
    password: "",
};

export function getRegistrationErrorMessage(error) {
    const status = error?.response?.status;
    if (status === 403) {
        return "El registro no está disponible en este momento.";
    }
    if (status === 429) {
        return "Se alcanzó el límite de registros. Inténtalo de nuevo más tarde.";
    }

    const data = error?.response?.data;
    if (typeof data === "string" && data.trim()) {
        return data;
    }

    const validationMessage = Object.values(data?.errors || {})
        .flat()
        .find((message) => typeof message === "string" && message.trim());

    return (
        data?.detail || data?.message || validationMessage || data?.title || "No fue posible crear la cuenta."
    );
}

function JWTRegisterTab() {
    const { control, formState, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues,
        resolver: yupResolver(registrationSchema),
    });

    const { isValid, dirtyFields, errors } = formState;
    const [showPassword, setShowPassword] = useState(false);
    const [submitting, setSubmitting] = useState(false);
    const [requestError, setRequestError] = useState("");

    async function onSubmit(model) {
        setSubmitting(true);
        setRequestError("");

        try {
            await axios.post(`${URL}/Accounts`, model);
            history.push("/login", { accountCreated: true });
        } catch (error) {
            setRequestError(getRegistrationErrorMessage(error));
            setSubmitting(false);
        }
    }

    return (
        <div className="w-full">
            <form
                className="flex flex-col justify-center w-full"
                onChange={() => requestError && setRequestError("")}
                onSubmit={handleSubmit(onSubmit)}
                aria-busy={submitting}
                noValidate
            >
                {requestError ? (
                    <Alert className="mb-16" severity="error" role="alert">
                        {requestError}
                    </Alert>
                ) : null}

                <Controller
                    name="firstName"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="text"
                            error={!!errors.firstName}
                            helperText={errors?.firstName?.message}
                            label="Nombre"
                            autoComplete="given-name"
                            disabled={submitting}
                            InputProps={{
                                endAdornment: (
                                    <InputAdornment position="end">
                                        <Icon className="text-20" color="action">
                                            person
                                        </Icon>
                                    </InputAdornment>
                                ),
                            }}
                            variant="outlined"
                            required
                        />
                    )}
                />

                <Controller
                    name="lastName"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="text"
                            error={!!errors.lastName}
                            helperText={errors?.lastName?.message}
                            label="Apellido"
                            autoComplete="family-name"
                            disabled={submitting}
                            InputProps={{
                                endAdornment: (
                                    <InputAdornment position="end">
                                        <Icon className="text-20" color="action">
                                            person
                                        </Icon>
                                    </InputAdornment>
                                ),
                            }}
                            variant="outlined"
                            required
                        />
                    )}
                />

                <Controller
                    name="email"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="email"
                            error={!!errors.email}
                            helperText={errors?.email?.message}
                            label="Correo electrónico"
                            autoComplete="email"
                            disabled={submitting}
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
                            required
                        />
                    )}
                />

                <Controller
                    name="password"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            label="Contraseña"
                            type={showPassword ? "text" : "password"}
                            error={!!errors.password}
                            helperText={errors?.password?.message || passwordMessage}
                            variant="outlined"
                            autoComplete="new-password"
                            disabled={submitting}
                            InputProps={{
                                className: "pr-2",
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
                    aria-label="Crear cuenta"
                    disabled={submitting || _.isEmpty(dirtyFields) || !isValid}
                    value="legacy"
                >
                    {submitting ? (
                        <>
                            <CircularProgress className="mr-8" color="inherit" size={18} />
                            Creando cuenta…
                        </>
                    ) : (
                        "Crear cuenta"
                    )}
                </Button>
            </form>
        </div>
    );
}

export default JWTRegisterTab;
