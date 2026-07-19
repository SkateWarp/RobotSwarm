import { motion } from "framer-motion";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Icon from "@mui/material/Icon";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { makeStyles } from "@mui/styles";
import { darken } from "@mui/material/styles";
import JWTLoginTab from "./tabs/JWTLoginTab";

const useStyles = makeStyles((theme) => ({
    root: {
        background: `radial-gradient(circle at top left, ${theme.palette.primary.main} 0%, ${darken(
            theme.palette.primary.dark,
            0.55
        )} 72%)`,
        color: theme.palette.primary.contrastText,
    },
    card: {
        border: "1px solid rgba(255, 255, 255, 0.18)",
        borderRadius: 20,
        boxShadow: "0 24px 64px rgba(8, 20, 36, 0.34)",
    },
    brandIcon: {
        alignItems: "center",
        backgroundColor: theme.palette.primary.main,
        borderRadius: 18,
        color: theme.palette.primary.contrastText,
        display: "flex",
        fontSize: 38,
        height: 72,
        justifyContent: "center",
        width: 72,
    },
}));

function Login() {
    const classes = useStyles();

    return (
        <div
            className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}
        >
            <div className="flex flex-col items-center justify-center w-full">
                <motion.div
                    className="w-full"
                    style={{ maxWidth: 420 }}
                    initial={{ opacity: 0, y: 18 }}
                    animate={{ opacity: 1, y: 0 }}
                >
                    <Card className={clsx(classes.card, "w-full")}>
                        <CardContent className="flex flex-col items-center justify-center p-24 sm:p-32 md:p-40">
                            <div className={clsx(classes.brandIcon, "mb-16")} aria-hidden="true">
                                <Icon fontSize="inherit">device_hub</Icon>
                            </div>

                            <Typography component="h1" variant="h5" className="font-oswald text-24 sm:text-28">
                                RobotSwarm
                            </Typography>
                            <Typography color="textSecondary" align="center" className="mt-8 mb-24 text-14">
                                Control y supervisión de simulaciones multi-robot
                            </Typography>
                            <JWTLoginTab />
                            <Typography
                                variant="caption"
                                align="center"
                                className="mt-24"
                                color="textSecondary"
                            >
                                El acceso se habilita mediante una cuenta creada por el administrador.
                            </Typography>
                        </CardContent>
                    </Card>
                </motion.div>
            </div>
        </div>
    );
}

export default Login;
