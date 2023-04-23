import PropTypes from "prop-types";
import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from "@mui/material";

const DeleteRowElementDialog = ({ isOpen, setIsOpen, deleteData, message }) => {
    const handleClose = () => {
        setIsOpen(false);
    };

    const handleDelete = (event) => {
        event.stopPropagation();
        deleteData();
        handleClose();
    };

    return (
        <Dialog
            open={isOpen}
            onClose={handleClose}
            aria-labelledby="alert-dialog-title"
            aria-describedby="alert-dialog-description"
        >
            <DialogTitle id="alert-dialog-title">Desea eliminar {message}?</DialogTitle>
            <DialogContent>
                <DialogContentText id="alert-dialog-description">
                    Al "Confirmar" usted estará eliminando {message}.
                </DialogContentText>
            </DialogContent>
            <DialogActions>
                <Button onClick={handleClose} color="primary">
                    Cancelar
                </Button>
                <Button onClick={handleDelete} color="primary" autoFocus>
                    Confirmar
                </Button>
            </DialogActions>
        </Dialog>
    );
};

DeleteRowElementDialog.propTypes = {
    isOpen: PropTypes.bool.isRequired,
    setIsOpen: PropTypes.func.isRequired,
    deleteData: PropTypes.func.isRequired,
    message: PropTypes.string.isRequired,
};

export default DeleteRowElementDialog;
