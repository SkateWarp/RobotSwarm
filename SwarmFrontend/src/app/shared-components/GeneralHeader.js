import PropTypes from "prop-types";
import { motion } from "framer-motion";
import { ThemeProvider } from "@mui/material/styles";
import { useDispatch, useSelector } from "react-redux";
import { Hidden, Icon, IconButton, Input, Paper, Typography } from "@mui/material";
import { selectMainTheme } from "../store/fuse/settingsSlice";

// Componente general de header para todos los modulos que realicen la busqueda mediante la paginacion
const GeneralHeader = ({ searchText, handleSearchTextChange, pageLayout, headerName, iconType }) => {
    const dispatch = useDispatch();
    const mainTheme = useSelector(selectMainTheme);

    return (
        <div className="flex flex-1 items-center justify-between p-4 sm:p-24">
            <div className="flex shrink items-center sm:w-2/24">
                <Hidden lgUp>
                    <IconButton
                        onClick={() => {
                            pageLayout.current.toggleLeftSidebar();
                        }}
                        aria-label="open left sidebar"
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
                        size="large"
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

            <div className="flex flex-1 items-center justify-center px-8 sm:px-12">
                <ThemeProvider theme={mainTheme}>
                    <Paper
                        component={motion.div}
                        initial={{ y: -20, opacity: 0 }}
                        animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                        className="flex p-4 items-center w-full max-w-512 h-48 px-16 py-4 shadow"
                    >
                        <Icon color="action">search</Icon>

                        <Input
                            placeholder="Buscar"
                            className="flex flex-1 px-16"
                            disableUnderline
                            fullWidth
                            value={searchText}
                            inputProps={{
                                "aria-label": "Search",
                            }}
                            onChange={(ev) => dispatch(handleSearchTextChange(ev))}
                        />
                    </Paper>
                </ThemeProvider>
            </div>
        </div>
    );
};

GeneralHeader.propTypes = {
    searchText: PropTypes.string.isRequired,
    handleSearchTextChange: PropTypes.func.isRequired,
    headerName: PropTypes.string.isRequired,
    pageLayout: PropTypes.object.isRequired,
    iconType: PropTypes.string.isRequired,
};

export default GeneralHeader;
