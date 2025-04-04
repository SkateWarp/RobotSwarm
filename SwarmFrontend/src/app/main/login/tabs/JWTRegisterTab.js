import TextField from "@mui/material/TextField";
import Button from "@mui/material/Button";
import Icon from "@mui/material/Icon";
import IconButton from "@mui/material/IconButton";
import InputAdornment from "@mui/material/InputAdornment";
import { useState } from "react";
import { Controller, useForm } from "react-hook-form";
import _ from "@lodash";
import axios from "axios";
import history from '@history';
import {URL} from "../../../constants/constants";

const defaultValues = {
    firstName: "",
    lastName: "",
    email: "",
    password: "",
};

function JWTRegisterTab() {
    const { control, formState, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues
    });

    const { isValid, dirtyFields, errors } = formState;

    const [showPassword, setShowPassword] = useState(false);

    function onSubmit(model) {

        axios.post(`${URL}/Accounts/`, model).then(() => {
            history.push({
                pathname: '/',
            });
        });
    }

    return (
        <div className="w-full">
            <form className="flex flex-col justify-center w-full" onSubmit={handleSubmit(onSubmit)}>

                <Controller
                    name="firstName"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="text"
                            error={!!errors.email}
                            helperText={errors?.email?.message}
                            label="Nombre"
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
                    name="lastName"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="mb-16"
                            type="text"
                            error={!!errors.email}
                            helperText={errors?.email?.message}
                            label="Apellido"
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
                    Registrar
                </Button>
            </form>
        </div>
    );
}

export default JWTRegisterTab;
