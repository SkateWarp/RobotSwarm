import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { makeStyles } from "@mui/styles";
import { LOGO } from "../../constants/constants";
import JWTRegisterTab from "./tabs/JWTRegisterTab";

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

function Register() {
    const classes = useStyles();

    return (
        <div
            className={clsx(classes.root, "flex flex-col flex-auto items-center justify-center p-16 sm:p-32")}
        >
            <main className="w-full" style={{ maxWidth: 420 }}>
                <header className={classes.brand}>
                    <img className={classes.brandLogo} src={LOGO} alt="RobotSwarm" />
                    <Typography color="textSecondary" variant="body2" className="mt-8">
                        Administración de acceso
                    </Typography>
                </header>

                <Card className={clsx(classes.card, "w-full")} variant="outlined">
                    <CardContent className="p-24 sm:p-32">
                        <Typography component="h1" variant="h6" className="font-semibold mb-8">
                            Crear cuenta
                        </Typography>
                        <Typography color="textSecondary" variant="body2" className="mb-24">
                            Registre las credenciales y el rol que utilizará la cuenta.
                        </Typography>
                        <JWTRegisterTab />
                    </CardContent>
                </Card>
            </main>
        </div>
    );
}

export default Register;
