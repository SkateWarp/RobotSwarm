import { motion } from "framer-motion";
import Alert from "@mui/material/Alert";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { Link, useLocation } from "react-router-dom";
import { makeStyles } from "@mui/styles";
import { darken } from "@mui/material/styles";
import { LOGO } from "../../constants/constants";
import JWTLoginTab from "./tabs/JWTLoginTab";

const useStyles = makeStyles((theme) => ({
    root: {
        background: `linear-gradient(to left, ${theme.palette.primary.dark} 0%, ${darken(
            theme.palette.primary.dark,
            0.5
        )} 100%)`,
        color: theme.palette.primary.contrastText,
    },
}));

function Login() {
    const classes = useStyles();
    const location = useLocation();
    const accountCreated = location.state?.accountCreated === true;

    return (
        <div
            className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}
        >
            <div className="flex flex-col items-center justify-center w-full">
                <motion.div initial={{ opacity: 0, scale: 0.6 }} animate={{ opacity: 1, scale: 1 }}>
                    <Card className="w-full max-w-384">
                        <CardContent className="flex flex-col items-center justify-center p-16 sm:p-24 md:p-32">
                            <img className="w-128 m-32" src={LOGO} alt="RobotSwarm" />

                            <Typography
                                component="h1"
                                variant="h6"
                                className="mt-16 mb-24 font-oswald text-18 sm:text-24"
                            >
                                BIENVENIDO
                            </Typography>
                            {accountCreated ? (
                                <Alert className="w-full mb-16" severity="success">
                                    Cuenta creada. Ya puedes iniciar sesión.
                                </Alert>
                            ) : null}
                            <JWTLoginTab />
                            <div className="flex flex-col items-center justify-center pt-32 pb-24">
                                <span className="font-normal">¿Olvidaste tu contraseña?</span>
                                <Link
                                    className="inline-flex items-center min-h-24 font-normal"
                                    to="/forgot-password"
                                >
                                    Restablecer contraseña
                                </Link>
                                <span className="font-normal mt-16">¿No tienes una cuenta?</span>
                                <Link className="inline-flex items-center min-h-24 font-normal" to="/register">
                                    Crear cuenta
                                </Link>
                            </div>
                            <div className="w-1/3 mt-32">
                                <img src={LOGO} alt="RobotSwarm" />
                            </div>
                            <Typography
                                variant="caption"
                                className="mt-16 text-center"
                                style={{ fontSize: "12px", color: "#999999" }}
                            >
                                © RobotSwarm 2022. Todos los derechos reservados.
                            </Typography>
                        </CardContent>
                    </Card>
                </motion.div>
            </div>
        </div>
    );
}

export default Login;
