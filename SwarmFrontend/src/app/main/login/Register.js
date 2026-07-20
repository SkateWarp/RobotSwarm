import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Icon from "@mui/material/Icon";
import Typography from "@mui/material/Typography";
import clsx from "clsx";
import { makeStyles } from "@mui/styles";
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
        alignItems: "center",
        display: "flex",
        gap: 12,
        marginBottom: 20,
    },
    brandIcon: {
        alignItems: "center",
        border: `1px solid ${theme.palette.divider}`,
        borderRadius: 6,
        color: theme.palette.primary.main,
        display: "flex",
        fontSize: 26,
        height: 44,
        justifyContent: "center",
        width: 44,
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
                    <div className={classes.brandIcon} aria-hidden="true">
                        <Icon fontSize="inherit">device_hub</Icon>
                    </div>
                    <div>
                        <Typography component="h1" variant="h5" className="font-semibold">
                            RobotSwarm
                        </Typography>
                        <Typography color="textSecondary" variant="body2">
                            Administración de acceso
                        </Typography>
                    </div>
                </header>

                <Card className={clsx(classes.card, "w-full")} variant="outlined">
                    <CardContent className="p-24 sm:p-32">
                        <Typography component="h2" variant="h6" className="font-semibold mb-8">
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
