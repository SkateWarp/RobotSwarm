import { yupResolver } from "@hookform/resolvers/yup";
import TextField from "@mui/material/TextField";
import Button from "@mui/material/Button";
import Icon from "@mui/material/Icon";
import IconButton from "@mui/material/IconButton";
import InputAdornment from "@mui/material/InputAdornment";
import { useEffect, useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { useDispatch, useSelector } from "react-redux";
import { submitLogin } from "app/auth/store/loginSlice";
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

    useEffect(() => {
        if (login.error && (login.error.email || login.error.password)) {
            formRef.current.updateInputsWithError({
                ...login.error,
            });
        }
    }, [login.error]);

    function onSubmit(model) {
        dispatch(submitLogin(model));
    }

    return (
        <div className="w-full">
            <form className="flex flex-col justify-center w-full" onSubmit={handleSubmit(onSubmit)}>
                <Controller
                    name="email"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="text"
                            error={!!errors.email}
                            helperText={errors?.email?.message}
                            label="Correo Electrónico"
                            autoComplete="current-email"
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
                            className="mb-16"
                            label="Contraseña"
                            type="password"
                            error={!!errors.password}
                            helperText={errors?.password?.message}
                            variant="outlined"
                            autoComplete="current-password"
                            InputProps={{
                                className: "pr-2",
                                type: showPassword ? "text" : "password",
                                endAdornment: (
                                    <InputAdornment position="end">
                                        <IconButton
                                            onClick={() => setShowPassword(!showPassword)}
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
                    className="w-full mx-auto mt-16 uppercase"
                    aria-label="LOG IN"
                    disabled={_.isEmpty(dirtyFields) || !isValid}
                    value="legacy"
                >
                    Iniciar Sesión
                </Button>
            </form>
        </div>
    );
}

export default JWTLoginTab;
