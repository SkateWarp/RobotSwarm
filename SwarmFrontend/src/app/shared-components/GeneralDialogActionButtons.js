import PropTypes from "prop-types";
import { Button, DialogActions } from "@mui/material";

const GeneralDialogActionButtons = ({ dialogType, isValid }) => {
    return (
        <>
            {dialogType === "new" ? (
                <DialogActions className="justify-between p-4 pb-16">
                    <div className="px-16">
                        <Button variant="contained" color="secondary" type="submit" disabled={!isValid}>
                            Crear
                        </Button>
                    </div>
                </DialogActions>
            ) : (
                <DialogActions className="justify-between p-4 pb-16">
                    <div className="px-16">
                        <Button variant="contained" color="secondary" type="submit" disabled={!isValid}>
                            Guardar
                        </Button>
                    </div>
                </DialogActions>
            )}
        </>
    );
};

GeneralDialogActionButtons.propTypes = {
    dialogType: PropTypes.string.isRequired,
    isValid: PropTypes.bool.isRequired,
};

export default GeneralDialogActionButtons;
