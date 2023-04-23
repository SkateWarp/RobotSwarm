import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import Icon from '@mui/material/Icon';
import Typography from '@mui/material/Typography';
import { motion } from 'framer-motion';
import { Link } from 'react-router-dom';
import {makeStyles} from "@mui/styles";
import clsx from "clsx";
import {darken} from "@mui/material/styles";
//import classes from "*.module.css";


const useStyles = makeStyles(theme => ({
    root: {
        background: `radial-gradient(${darken(theme.palette.primary.dark, 0.5)} 0%, ${theme.palette.primary.dark} 80%)`,
        color: theme.palette.primary.contrastText
    }
}));

function MailConfirmPage() {

    const classes = useStyles();
    return (
        <div className={clsx(classes.root, 'flex flex-col flex-auto flex-shrink-0 items-center justify-center p-32')}>
            <div className="flex flex-col items-center justify-center w-full">
                <motion.div initial={{ opacity: 0, scale: 0.6 }} animate={{ opacity: 1, scale: 1 }}>
                    <Card className="w-full max-w-384">
                        <CardContent className="flex flex-col items-center justify-center p-16 sm:p-24 md:p-32">
                              <div className="m-32">
                                <Icon className="text-96" color="action">
                                    email
                                </Icon>
                               </div>

                            <Typography variant="h5" className="text-center mb-16 font-semibold">
                                ¡Confirma tu correo electronico!
                            </Typography>

                            <Typography className="text-center mb-16 w-full" color="textSecondary">
                                Un correo de confirmación ha sido enviado a <b>ejemplo@micorreo.com</b>.
                            </Typography>

                            <Typography className="text-center w-full" color="textSecondary">
                                Verifique su bandeja de entrada y haga clic en el enlace "Confirmar mi correo" para poder verificar su correo
                                electronico.
                            </Typography>

                            <div className="flex flex-col items-center justify-center pt-32 pb-24">
                                <Link className="font-normal" to="/pages/auth/login">
                                    Ir a iniciar sesión
                                </Link>
                            </div>
                        </CardContent>
                    </Card>
                </motion.div>
            </div>
        </div>
    );
}

export default MailConfirmPage;
