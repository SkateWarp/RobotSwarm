import { Hidden, Icon, IconButton, Typography, Button } from "@mui/material";
import PropTypes from "prop-types";

const SimpleGeneralHeader = ({ pageLayout, headerName, iconType, actionButton, hasSidebar = true }) => {
    return (
        <div className="flex flex-1 items-center justify-between gap-16 px-12 sm:px-24">
            <div className="flex flex-shrink items-center min-w-0">
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
                    <Icon className="text-24" color="action">
                        {iconType}
                    </Icon>
                    <Typography
                        component="h1"
                        className="text-18 md:text-20 mx-12 font-semibold"
                    >
                        {headerName}
                    </Typography>
                </div>
            </div>

            {actionButton && (
                <Button
                    variant="contained"
                    color="secondary"
                    onClick={actionButton.onClick}
                    startIcon={actionButton.icon && <Icon>{actionButton.icon}</Icon>}
                >
                    {actionButton.text}
                </Button>
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
