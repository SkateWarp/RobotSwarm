import Avatar from "@mui/material/Avatar";
import ListItem from "@mui/material/ListItem";
import Typography from "@mui/material/Typography";
import formatDistanceToNow from "date-fns/formatDistanceToNow";
import Box from "@mui/material/Box";
import IconButton from "@mui/material/IconButton";
import {
    Button,
    Dialog,
    DialogActions,
    DialogContent,
    DialogContentText,
    DialogTitle,
    Icon,
} from "@mui/material";
import { useState } from "react";
import { es } from "date-fns/locale";

function CardActivity(props) {
    const [open, setOpen] = useState(false);

    const handleOpen = () => {
        setOpen(true);
    };

    const handleClose = () => {
        setOpen(false);
    };

    return (
        <ListItem dense className="px-0">
            <Avatar alt={props.firstName} className="w-32 h-32" />
            <Box
                className="flex flex-col mx-16 p-12"
                sx={{
                    borderRadius: "5px 20px 20px 5px",
                    border: (theme) => `1px solid ${theme.palette.divider}`,
                }}
            >
                <div className="flex items-center">
                    <Typography className="font-semibold">
                        {props.firstName} {props.lastName}
                    </Typography>
                    <Typography className="mx-8 text-12" color="textSecondary">
                        {formatDistanceToNow(Date.parse(props.time), { addSuffix: true, locale: es })}
                    </Typography>
                </div>
                <Typography>
                    {props.item}
                    <IconButton onClick={handleOpen} size="small">
                        <Icon>cancel</Icon>
                    </IconButton>
                    <Dialog
                        open={open}
                        onClose={handleClose}
                        aria-labelledby="alert-dialog-title"
                        aria-describedby="alert-dialog-description"
                    >
                        <DialogTitle id="alert-dialog-title">¿Desea eliminar este comentario?</DialogTitle>
                        <DialogContent>
                            <DialogContentText id="alert-dialog-description">
                                Al "Confirmar" usted estará eliminando este comentario.
                            </DialogContentText>
                        </DialogContent>
                        <DialogActions>
                            <Button onClick={handleClose} color="primary">
                                Cancelar
                            </Button>
                            <Button onClick={() => props.onDelete(props.id)} color="primary" autoFocus>
                                Confirmar
                            </Button>
                        </DialogActions>
                    </Dialog>
                </Typography>
            </Box>
        </ListItem>
    );
}

export default CardActivity;
