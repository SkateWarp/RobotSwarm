import {
    Alert,
    Box,
    Button,
    Icon,
    IconButton,
    LinearProgress,
    Paper,
    Table,
    TableBody,
    TableCell,
    TableContainer,
    TableHead,
    TableRow,
    Tooltip,
    Typography,
} from "@mui/material";
import moment from "moment";
import { useDispatch, useSelector } from "react-redux";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";
import { taskTemplateTypeLabel } from "./taskTemplateModel";
import { getTaskTemplates, openTaskTemplateDialog, selectTaskTemplates } from "./store/leafTypeConfigSlice";

function LeafTypesConfigList() {
    const dispatch = useDispatch();
    const templates = useSelector(selectTaskTemplates);
    const { error, loading } = useSelector(({ leafTypesConfigApp }) => leafTypesConfigApp.leafTypes);

    const retry = () => dispatch(getTaskTemplates());

    if (loading && templates.length === 0) {
        return <ChargingProgressBar />;
    }

    return (
        <Box className="flex flex-col flex-auto w-full min-h-full gap-12">
            <Box
                className="flex flex-col sm:flex-row sm:items-center justify-between gap-12 pb-12"
                sx={{ borderBottom: 1, borderColor: "divider" }}
            >
                <Box>
                    <Typography variant="body1">{templates.length} plantillas disponibles</Typography>
                    <Typography color="textSecondary" variant="body2">
                        Este catálogo clasifica algoritmos; no inicia tareas ni selecciona robots.
                    </Typography>
                </Box>
                <Button disabled={loading} onClick={retry} startIcon={<Icon>refresh</Icon>} variant="outlined">
                    Actualizar
                </Button>
            </Box>

            {loading ? <LinearProgress aria-label="Actualizando plantillas" /> : null}

            {error ? (
                <Alert
                    action={
                        <Button color="inherit" disabled={loading} onClick={retry} size="small">
                            Reintentar
                        </Button>
                    }
                    severity="error"
                    role="alert"
                >
                    {error}
                </Alert>
            ) : null}

            {!error && templates.length === 0 ? (
                <Box className="flex flex-1 flex-col items-center justify-center min-h-256 text-center p-24">
                    <Icon className="text-48 mb-12" color="disabled">
                        assignment
                    </Icon>
                    <Typography variant="h6">No hay plantillas configuradas</Typography>
                    <Typography color="textSecondary" className="mt-4 mb-16">
                        El catálogo está vacío. Puede volver a consultar el backend sin salir de la página.
                    </Typography>
                    <Button disabled={loading} onClick={retry} variant="outlined">
                        Volver a consultar
                    </Button>
                </Box>
            ) : null}

            {templates.length > 0 ? (
                <TableContainer component={Paper} elevation={0} variant="outlined">
                    <Table aria-label="Plantillas de tareas" size="small" stickyHeader>
                        <TableHead>
                            <TableRow>
                                <TableCell>Nombre</TableCell>
                                <TableCell>Tipo</TableCell>
                                <TableCell>Fecha de creación</TableCell>
                                <TableCell align="center">Acciones</TableCell>
                            </TableRow>
                        </TableHead>
                        <TableBody>
                            {templates.map((template) => (
                                <TableRow key={template.id} hover>
                                    <TableCell component="th" scope="row">
                                        {template.name}
                                    </TableCell>
                                    <TableCell>{taskTemplateTypeLabel(template)}</TableCell>
                                    <TableCell>{moment(template.dateCreated).format("DD-MM-YYYY")}</TableCell>
                                    <TableCell align="center">
                                        <Tooltip title="Editar plantilla">
                                            <IconButton
                                                aria-label={`Editar la plantilla ${template.name}`}
                                                onClick={() => dispatch(openTaskTemplateDialog(template))}
                                                size="large"
                                            >
                                                <Icon>edit</Icon>
                                            </IconButton>
                                        </Tooltip>
                                    </TableCell>
                                </TableRow>
                            ))}
                        </TableBody>
                    </Table>
                </TableContainer>
            ) : null}
        </Box>
    );
}

export default LeafTypesConfigList;
