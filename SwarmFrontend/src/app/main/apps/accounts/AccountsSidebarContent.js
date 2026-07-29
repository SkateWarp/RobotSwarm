import { motion } from "framer-motion";
import { useDispatch } from "react-redux";
import { Button, Icon, Paper, Typography } from "@mui/material";
import { openNewAccountDialog } from "./store/accountsSlice";

function AccountsSidebarContent() {
    const dispatch = useDispatch();

    return (
        <div className="p-0 lg:p-24 lg:ltr:pr-4 lg:rtl:pl-4">
            <Paper
                component={motion.div}
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.15 } }}
                className="rounded-0 shadow-none lg:rounded-16 lg:shadow p-20"
            >
                <Button
                    aria-label="Crear una cuenta"
                    className="w-full"
                    color="secondary"
                    onClick={() => dispatch(openNewAccountDialog())}
                    startIcon={<Icon>person_add</Icon>}
                    variant="contained"
                >
                    Crear cuenta
                </Button>
                <Typography className="mt-16" color="textSecondary" variant="body2">
                    El administrador crea y gestiona el acceso de cada usuario. Las cuentas desactivadas no
                    pueden iniciar sesión hasta que un administrador las reactive.
                </Typography>
            </Paper>
        </div>
    );
}

export default AccountsSidebarContent;
