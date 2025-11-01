import { motion } from "framer-motion";
import { Hidden, Icon, IconButton, Typography, Button } from "@mui/material";
import PropTypes from "prop-types";

const SimpleGeneralHeader = ({ pageLayout, headerName, iconType, actionButton, hasSidebar = true }) => {
    return (
        <div className="flex flex-1 items-center justify-between p-4 sm:p-24">
            <div className="flex flex-shrink items-center w-1/2">
                {hasSidebar && (
                    <Hidden lgUp>
                        <IconButton
                            onClick={() => {
                                pageLayout.current.toggleLeftSidebar();
                            }}
                            aria-label="open left sidebar"
                            size="large"
                        >
                            <Icon>menu</Icon>
                        </IconButton>
                    </Hidden>
                )}

                <div className="flex items-center">
                    <Icon
                        component={motion.span}
                        initial={{ scale: 0 }}
                        animate={{ scale: 1, transition: { delay: 0.2 } }}
                        className="text-24 md:text-32"
                    >
                        {iconType}
                    </Icon>
                    <Typography
                        component={motion.span}
                        initial={{ x: -20 }}
                        animate={{ x: 0, transition: { delay: 0.2 } }}
                        delay={300}
                        className="hidden sm:flex text-16 md:text-24 mx-12 font-oswald uppercase"
                    >
                        {headerName}
                    </Typography>
                </div>
            </div>

            {actionButton && (
                <motion.div
                    initial={{ opacity: 0, x: 20 }}
                    animate={{ opacity: 1, x: 0, transition: { delay: 0.2 } }}
                >
                    <Button
                        variant="contained"
                        color="secondary"
                        onClick={actionButton.onClick}
                        startIcon={actionButton.icon && <Icon>{actionButton.icon}</Icon>}
                    >
                        {actionButton.text}
                    </Button>
                </motion.div>
            )}
        </div>
    );
};

SimpleGeneralHeader.propTypes = {
    headerName: PropTypes.string.isRequired,
    pageLayout: PropTypes.object.isRequired,
    iconType: PropTypes.string.isRequired,
    actionButton: PropTypes.shape({
        text: PropTypes.string.isRequired,
        onClick: PropTypes.func.isRequired,
        icon: PropTypes.string,
    }),
    hasSidebar: PropTypes.bool,
};

export default SimpleGeneralHeader;
