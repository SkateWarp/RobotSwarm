import PropTypes from "prop-types";
import { useDispatch } from "react-redux";
import { motion } from "framer-motion";
import { Button, Paper } from "@mui/material";

const SimpleSidebarContent = ({ buttonText, openDialogFunction }) => {
    const dispatch = useDispatch();

    return (
        <div className="p-0 lg:p-24 lg:ltr:pr-4 lg:rtl:pl-4">
            <Paper
                component={motion.div}
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                className="rounded-0 shadow-none lg:rounded-16 lg:shadow"
            >
                <div className="p-24">
                    <Button
                        variant="contained"
                        color="secondary"
                        className="w-full"
                        onClick={() => dispatch(openDialogFunction())}
                    >
                        {buttonText}
                    </Button>
                </div>
            </Paper>
        </div>
    );
};

SimpleSidebarContent.propTypes = {
    buttonText: PropTypes.string.isRequired,
    openDialogFunction: PropTypes.func.isRequired,
};

export default SimpleSidebarContent;
