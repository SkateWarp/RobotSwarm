import PropTypes from "prop-types";
import { ThemeProvider } from "@mui/material/styles";
import { useDispatch, useSelector } from "react-redux";
import { Button, Hidden, Icon, IconButton, Input, Paper, Tooltip, Typography } from "@mui/material";
import { selectMainTheme } from "../store/fuse/settingsSlice";

// Componente general de header para todos los modulos que realicen la busqueda mediante la paginacion
const GeneralHeader = ({
    searchText,
    handleSearchTextChange,
    pageLayout,
    headerName,
    iconType,
    actionButton,
    hasSidebar,
}) => {
    const dispatch = useDispatch();
    const mainTheme = useSelector(selectMainTheme);

    return (
        <div className="flex flex-1 items-center justify-between gap-16 px-12 sm:px-24">
            <div className="flex shrink-0 items-center">
                {hasSidebar ? (
                    <Hidden lgUp>
                        <IconButton
                            onClick={() => {
                                pageLayout.current.toggleLeftSidebar();
                            }}
                            aria-label="Abrir panel lateral"
                        >
                            <Icon>menu</Icon>
                        </IconButton>
                    </Hidden>
                ) : null}

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

            <div className="flex flex-1 items-center justify-center px-8 sm:px-12">
                <ThemeProvider theme={mainTheme}>
                    <Paper
                        className="flex items-center w-full max-w-512 h-40 px-12"
                        elevation={0}
                        variant="outlined"
                    >
                        <Icon color="action">search</Icon>

                        <Input
                            placeholder="Buscar"
                            className="flex flex-1 px-12"
                            disableUnderline
                            fullWidth
                            value={searchText}
                            inputProps={{
                                "aria-label": `Buscar en ${headerName.toLocaleLowerCase("es")}`,
                            }}
                            onChange={(ev) => dispatch(handleSearchTextChange(ev))}
                        />
                    </Paper>
                </ThemeProvider>
            </div>

            {actionButton ? (
                <>
                    <Hidden mdUp>
                        <Tooltip title={actionButton.text}>
                            <IconButton aria-label={actionButton.text} onClick={actionButton.onClick}>
                                <Icon>{actionButton.icon}</Icon>
                            </IconButton>
                        </Tooltip>
                    </Hidden>
                    <Hidden smDown>
                        <Button
                            color="secondary"
                            onClick={actionButton.onClick}
                            startIcon={<Icon>{actionButton.icon}</Icon>}
                            variant="contained"
                        >
                            {actionButton.text}
                        </Button>
                    </Hidden>
                </>
            ) : null}
        </div>
    );
};

GeneralHeader.propTypes = {
    searchText: PropTypes.string.isRequired,
    handleSearchTextChange: PropTypes.func.isRequired,
    headerName: PropTypes.string.isRequired,
    pageLayout: PropTypes.object.isRequired,
    iconType: PropTypes.string.isRequired,
    actionButton: PropTypes.shape({
        text: PropTypes.string.isRequired,
        onClick: PropTypes.func.isRequired,
        icon: PropTypes.string.isRequired,
    }),
    hasSidebar: PropTypes.bool,
};

GeneralHeader.defaultProps = {
    actionButton: undefined,
    hasSidebar: true,
};

export default GeneralHeader;
