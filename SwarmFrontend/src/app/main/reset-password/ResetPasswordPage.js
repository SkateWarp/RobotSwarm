import {motion} from 'framer-motion';
import {Controller} from 'react-hook-form';
import Button from '@mui/material/Button';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import TextField from '@mui/material/TextField';
import Typography from '@mui/material/Typography';
import {Link} from 'react-router-dom';
import {InputAdornment} from '@mui/material';
import clsx from "clsx";
import Icon from '@mui/material/Icon';
import AppLogo from "./AppLogo";
import useResetPasswordFunctions from "./useResetPasswordFunctions";

function ResetPasswordPage() {

    const {classes, control, errors, canBeSubmitted, handleSubmit, onSubmit} = useResetPasswordFunctions();

    return (

        <div className={clsx(classes.root, 'flex flex-col flex-auto flex-shrink-0 items-center justify-center p-32')}>
            <motion.div initial={{opacity: 0, scale: 0.6}} animate={{opacity: 1, scale: 1}}>
                <Card className="w-full max-w-384">
                    <CardContent className="flex flex-col items-center justify-center p-16 sm:p-24 md:p-32">

                        <AppLogo/>

                        <Typography variant="h6" className="mt-16 mb-24 font-semibold text-18 sm:text-24">
                            CAMBIA TU CONTRASEÑA
                        </Typography>

                        <form
                            name="resetForm"
                            noValidate
                            className="flex flex-col justify-center w-full"
                            onSubmit={handleSubmit(onSubmit)}
                        >

                            <Controller
                                name="password"
                                control={control}
                                render={({field}) => (
                                    <TextField
                                        {...field}
                                        className="mb-16"
                                        label="Contraseña"
                                        type="password"
                                        name="password"
                                        error={!!errors.password}
                                        helperText={errors?.password?.message}
                                        variant="outlined"
                                        required
                                        fullWidth
                                        InputProps={{
                                            endAdornment: (
                                                <InputAdornment position="end">
                                                    <Icon className="text-20" color="action">
                                                        vpn_key
                                                    </Icon>
                                                </InputAdornment>
                                            )
                                        }}
                                    />
                                )}
                            />

                            <Controller
                                name="confirmPassword"
                                control={control}
                                render={({field}) => (
                                    <TextField
                                        {...field}
                                        className="mb-16"
                                        label="Repita la contraseña"
                                        type="password"
                                        error={!!errors.confirmPassword}
                                        helperText={errors?.confirmPassword?.message}
                                        variant="outlined"
                                        required
                                        fullWidth
                                        InputProps={{
                                            endAdornment: (
                                                <InputAdornment position="end">
                                                    <Icon className="text-20" color="action">
                                                        vpn_key
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
                                disabled={!canBeSubmitted()}
                                type="submit"
                            >
                                Restablecer tu contraseña.
                            </Button>
                        </form>

                        <div className="flex flex-col items-center justify-center pt-32 pb-24">
                            <Link className="font-normal" to="/login">
                                Ir a iniciar sesión
                            </Link>
                        </div>
                    </CardContent>
                </Card>
            </motion.div>
        </div>
    );
}

export default ResetPasswordPage;
