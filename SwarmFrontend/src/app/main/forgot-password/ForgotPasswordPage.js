import Typography from '@mui/material/Typography';
import {motion} from 'framer-motion';
import {Controller} from 'react-hook-form';
import Button from '@mui/material/Button';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import TextField from '@mui/material/TextField';
import {Link} from 'react-router-dom';
import _ from '@lodash';
import clsx from "clsx";
import InputAdornment from "@mui/material/InputAdornment";
import {Icon} from "@mui/material";
import AppLogo from "../reset-password/AppLogo";
import useForgotPasswordFunctions from "./useForgotPasswordFunctions";

function ForgotPasswordPage() {

    const {classes, control, formState: {isValid, dirtyFields, errors},
        handleSubmit, onSubmit} = useForgotPasswordFunctions();

    return (
        <div className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}>
            <div className="flex flex-col items-center justify-center w-full">
                <motion.div initial={{opacity: 0, scale: 0.6}} animate={{opacity: 1, scale: 1}}>
                    <Card className="w-full max-w-384">
                        <CardContent className="flex flex-col items-center justify-center p-16 sm:p-24 md:p-32">

                            <AppLogo/>

                            <Typography variant="h6" className="mt-16 mb-24 font-semibold text-18 sm:text-24">
                                RECUPERAR CONTRASEÑA
                            </Typography>

                            <form
                                name="recoverForm"
                                noValidate
                                className="flex flex-col justify-center w-full"
                                onSubmit={handleSubmit(onSubmit)}
                            >
                                <Controller
                                    name="email"
                                    control={control}
                                    render={({field}) => (
                                        <TextField
                                            {...field}
                                            className="mb-16"
                                            label="Correo Electrónico"
                                            autoFocus
                                            type="email"
                                            error={!!errors.email}
                                            helperText={errors?.email?.message}
                                            variant="outlined"
                                            fullWidth
                                            InputProps={{
                                                endAdornment: (
                                                    <InputAdornment position="end">
                                                        <Icon className="text-20" color="action">
                                                            email
                                                        </Icon>
                                                    </InputAdornment>
                                                )
                                            }}
                                        />
                                    )}
                                />

                                <Button
                                    variant="contained"
                                    color="primary"
                                    className="w-224 mx-auto mt-16"
                                    aria-label="Reset"
                                    disabled={_.isEmpty(dirtyFields) || !isValid}
                                    type="submit"
                                >
                                    ENVIAR ENLACE
                                </Button>
                            </form>

                            <div className="flex flex-col items-center justify-center pt-32 pb-24">
                                <span className="font-normal">¿Ya tienes una cuenta?</span>
                                <Link className="font-normal" to="/login">
                                    Iniciar Sesión
                                </Link>
                            </div>
                        </CardContent>
                    </Card>
                </motion.div>
            </div>
        </div>
    );
}

export default ForgotPasswordPage;
