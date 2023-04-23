import { motion } from "framer-motion";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { Link } from "react-router-dom";
// import * as yup from 'yup';
import { makeStyles } from "@mui/styles";
import { darken } from "@mui/material/styles";
import JWTLoginTab from "./tabs/JWTLoginTab";
import { LOGO } from "../../constants/constants";
import settingsConfig from "../../fuse-configs/settingsConfig";

/**
 * Form Validation Schema
 */
// const schema = yup.object().shape({
//   email: yup.string().email('You must enter a valid email').required('You must enter a email'),
//   password: yup
//     .string()
//     .required('Please enter your password.')
//     .min(8, 'Password is too short - should be 8 chars minimum.'),
// });

const useStyles = makeStyles((theme) => ({
    root: {
        background: `linear-gradient(to left, ${theme.palette.primary.dark} 0%, ${darken(
            theme.palette.primary.dark,
            0.5
        )} 100%)`,
        color: theme.palette.primary.contrastText,
    },
    leftSection: {},
    rightSection: {
        background: `linear-gradient(to right, ${theme.palette.primary.dark} 0%, ${darken(
            theme.palette.primary.dark,
            0.5
        )} 100%)`,
        color: theme.palette.primary.contrastText,
    },
}));

// const defaultValues = {
//   email: '',
//   password: '',
//   remember: true,
// };

function Login() {
    const classes = useStyles();

    return (
        <div
            className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}
        >
            <div className="flex flex-col items-center justify-center w-full">
                <motion.div initial={{ opacity: 0, scale: 0.6 }} animate={{ opacity: 1, scale: 1 }}>
                    <Card className="w-full max-w-384">
                        <CardContent className="flex flex-col items-center justify-center p-16 sm:p-24 md:p-32">
                            <img className="w-128 m-32" src={LOGO} alt="logo" />

                            <Typography variant="h6" className="mt-16 mb-24 font-oswald text-18 sm:text-24">
                                BIENVENIDO
                            </Typography>
                            <JWTLoginTab />
                            <div className="flex flex-col items-center justify-center pt-32 pb-24">
                                <span className="font-normal">¿Olvidaste tu contraseña?</span>
                                {settingsConfig.layout.project === "fraga" ? (
                                    <Link
                                        style={{ color: "#006565" }}
                                        className="font-normal"
                                        to="/forgot-password"
                                    >
                                        Reestablecer Contraseña
                                    </Link>
                                ) : (
                                    <Link className="font-normal" to="/forgot-password">
                                        Reestablecer Contraseña
                                    </Link>
                                )}
                            </div>
                            <div className="w-1/3 mt-32">
                                <a className="" href="https://www.alternard.com">
                                    <img src="assets/images/logos/alterna.png" alt="logo" />
                                </a>
                            </div>
                            <Typography
                                variant="caption"
                                className="mt-16"
                                style={{ fontSize: "12px", color: "#999999" }}
                            >
                                © Alterna 4.0 2022 all rights reserved.
                            </Typography>
                        </CardContent>
                    </Card>
                </motion.div>
            </div>
        </div>
    );
}

export default Login;
