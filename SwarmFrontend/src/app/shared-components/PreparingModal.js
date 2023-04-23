import { useState } from "react";
import { makeStyles } from "@mui/styles";
import { Backdrop, Fade, Modal } from "@mui/material";
import PropTypes from "prop-types";
import ProgressBar from "./ProgressBar";

const useStyles = makeStyles((theme) => ({
    modal: {
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
    },
    paper: {
        backgroundColor: theme.palette.background.paper,
        border: "2px solid #000",
        boxShadow: theme.shadows[5],
        padding: theme.spacing(2, 4, 3),
    },
}));

export default function PreparingModal({ callBack }) {
    const classes = useStyles();
    const [open, setOpen] = useState(true);

    const handleClose = () => {
        setOpen(false);
    };

    return (
        <div>
            <Modal
                aria-labelledby="transition-modal-title"
                aria-describedby="transition-modal-description"
                className={classes.modal}
                open={open}
                onClose={handleClose}
                closeAfterTransition
                BackdropComponent={Backdrop}
                BackdropProps={{
                    timeout: 500,
                }}
            >
                <Fade in={open}>
                    <div className={classes.paper}>
                        <h2 id="transition-modal-title">Preparando para impresión...</h2>
                        <ProgressBar callBack={callBack} setOpen={setOpen} />
                    </div>
                </Fade>
            </Modal>
        </div>
    );
}

PreparingModal.propTypes = {
    callBack: PropTypes.func.isRequired,
};
