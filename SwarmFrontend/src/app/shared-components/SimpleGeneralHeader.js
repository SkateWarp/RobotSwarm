import { motion } from "framer-motion";
import { Hidden, Icon, IconButton, Typography } from "@mui/material";
import PropTypes from "prop-types";

const SimpleGeneralHeader = ({ pageLayout, headerName, iconType }) => {
    return (
        <div className="flex flex-1 items-center justify-between p-4 sm:p-24">
            <div className="flex flex-shrink items-center w-1/2">
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
        </div>
    );
};

SimpleGeneralHeader.propTypes = {
    headerName: PropTypes.string.isRequired,
    pageLayout: PropTypes.object.isRequired,
    iconType: PropTypes.string.isRequired,
};

export default SimpleGeneralHeader;
