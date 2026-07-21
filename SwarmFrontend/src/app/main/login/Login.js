import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { makeStyles } from "@mui/styles";
import { LOGO } from "../../constants/constants";
import JWTLoginTab from "./tabs/JWTLoginTab";

const useStyles = makeStyles((theme) => ({
    root: {
        backgroundColor: theme.palette.background.default,
        color: theme.palette.text.primary,
    },
    card: {
        border: `1px solid ${theme.palette.divider}`,
        borderRadius: 8,
        boxShadow: "none",
    },
    brand: {
        marginBottom: 20,
    },
    brandLogo: {
        display: "block",
        height: "auto",
        maxWidth: 220,
        width: "62%",
    },
}));

function Login() {
    const classes = useStyles();

    return (
        <div
            className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}
        >
            <main className="w-full" style={{ maxWidth: 420 }}>
                <header className={classes.brand}>
                    <img className={classes.brandLogo} src={LOGO} alt="RobotSwarm" />
                    <Typography color="textSecondary" variant="body2" className="mt-8">
                        Plataforma de simulación multi-robot
                    </Typography>
                </header>

                <Card className={clsx(classes.card, "w-full")} variant="outlined">
                    <CardContent className="p-24 sm:p-32">
                        <Typography component="h1" variant="h6" className="font-semibold mb-8">
                            Iniciar sesión
                        </Typography>
                        <Typography color="textSecondary" variant="body2" className="mb-24">
                            Utilice las credenciales asignadas por el administrador.
                        </Typography>
                            <JWTLoginTab />
                    </CardContent>
                </Card>
                <Typography variant="caption" color="textSecondary" className="block mt-16">
                    Acceso restringido. Cada sesión de simulación y visor pertenece a una sola cuenta.
                </Typography>
            </main>
        </div>
    );
}

export default Login;
