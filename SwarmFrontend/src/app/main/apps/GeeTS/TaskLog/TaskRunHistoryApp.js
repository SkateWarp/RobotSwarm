import { useCallback, useEffect, useMemo, useState } from "react";
import { useNavigate } from "react-router-dom";
import {
    Alert,
    Box,
    Button,
    Chip,
    CircularProgress,
    Dialog,
    DialogActions,
    DialogContent,
    DialogTitle,
    FormControl,
    IconButton,
    InputLabel,
    MenuItem,
    Paper,
    Select,
    Table,
    TableBody,
    TableCell,
    TableContainer,
    TableHead,
    TablePagination,
    TableRow,
    Tooltip,
    Typography,
} from "@mui/material";
import { Close, Refresh, Visibility } from "@mui/icons-material";
import SimulationSessionService from "../../../../../services/SimulationSessionService";
import PageHeading from "../../../../shared-components/PageHeading";

const TYPE_LABELS = {
    FollowLeader: "Seguir al líder",
    Figure: "Formación o figura",
    CollaborativeTransport: "Transporte colaborativo",
};

const STATE_LABELS = {
    Queued: "En cola",
    Accepted: "Aceptada",
    Running: "En ejecución",
    Paused: "En pausa",
    Cancelling: "Cancelando",
    Completed: "Terminada",
    Cancelled: "Cancelada",
    Failed: "Fallida",
};

const OUTCOME_LABELS = {
    Pending: "Pendiente",
    Succeeded: "Aprobada",
    Failed: "Fallida",
    Cancelled: "Cancelada",
};

export const taskTypeLabel = (type) => TYPE_LABELS[type] || type || "Tarea desconocida";
export const taskStateLabel = (state) => STATE_LABELS[state] || state || "Sin estado";
export const taskOutcomeLabel = (outcome) => OUTCOME_LABELS[outcome] || outcome || "Sin resultado";

export const taskStateColor = (state) => {
    if (state === "Completed") return "success";
    if (state === "Failed") return "error";
    if (state === "Cancelled") return "default";
    if (state === "Paused" || state === "Cancelling") return "warning";
    return "info";
};

export const taskOutcomeColor = (outcome) => {
    if (outcome === "Succeeded") return "success";
    if (outcome === "Failed") return "error";
    if (outcome === "Cancelled") return "default";
    return "info";
};

export const formatTaskDuration = (task) => {
    const start = Date.parse(task?.startedAt || task?.createdAt);
    const finish = Date.parse(task?.completedAt || task?.updatedAt);
    if (!Number.isFinite(start) || !Number.isFinite(finish) || finish < start) return "—";

    const totalSeconds = Math.round((finish - start) / 1000);
    const minutes = Math.floor(totalSeconds / 60);
    const seconds = totalSeconds % 60;
    return minutes > 0 ? `${minutes} min ${seconds} s` : `${seconds} s`;
};

export const formatTaskPayload = (payload) => {
    if (payload === null || payload === undefined) return "Sin datos";
    try {
        return JSON.stringify(payload, null, 2);
    } catch (_error) {
        return "No fue posible representar estos datos.";
    }
};

const formatDate = (value) => {
    const date = new Date(value);
    if (Number.isNaN(date.getTime())) return "—";
    return new Intl.DateTimeFormat("es-BO", {
        dateStyle: "short",
        timeStyle: "medium",
    }).format(date);
};

const requestMessage = (error) =>
    error?.response?.data?.detail ||
    error?.response?.data?.message ||
    error?.response?.data?.title ||
    "No fue posible cargar el historial de tareas.";

function TaskRunHistoryApp() {
    const navigate = useNavigate();
    const [items, setItems] = useState([]);
    const [total, setTotal] = useState(0);
    const [page, setPage] = useState(0);
    const [rowsPerPage, setRowsPerPage] = useState(10);
    const [type, setType] = useState("");
    const [state, setState] = useState("");
    const [outcome, setOutcome] = useState("");
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState("");
    const [selectedTask, setSelectedTask] = useState(null);

    const loadHistory = useCallback(async () => {
        setLoading(true);
        setError("");
        try {
            const response = await SimulationSessionService.listTaskHistory({
                offset: page * rowsPerPage,
                limit: rowsPerPage,
                type,
                state,
                outcome,
            });
            setItems(Array.isArray(response?.items) ? response.items : []);
            setTotal(Number(response?.total) || 0);
        } catch (requestError) {
            setItems([]);
            setTotal(0);
            setError(requestMessage(requestError));
        } finally {
            setLoading(false);
        }
    }, [outcome, page, rowsPerPage, state, type]);

    useEffect(() => {
        loadHistory();
    }, [loadHistory]);

    const filtersActive = Boolean(type || state || outcome);
    const summary = useMemo(() => {
        if (loading) return "Actualizando…";
        if (total === 1) return "1 tarea real de simulación";
        return `${total} tareas reales de simulación`;
    }, [loading, total]);

    const changeType = (event) => {
        setPage(0);
        setType(event.target.value);
    };

    const changeOutcome = (event) => {
        setPage(0);
        setOutcome(event.target.value);
    };

    const changeState = (event) => {
        setPage(0);
        setState(event.target.value);
    };

    return (
        <Box data-testid="task-history-page" sx={{ p: { xs: 2, md: 3 }, maxWidth: 1600, mx: "auto" }}>
            <PageHeading
                title="Historial de tareas"
                description="Resultados persistentes de las tareas ejecutadas en ROS y Gazebo."
                meta={summary}
                actions={
                    <Button variant="contained" onClick={() => navigate("/apps/GTS/realtime")}>
                        Abrir Control
                    </Button>
                }
            />

            <Paper elevation={0} variant="outlined" sx={{ p: 2, mb: 2 }}>
                <Box className="flex items-center" sx={{ gap: 2, flexWrap: "wrap" }}>
                    <FormControl size="small" sx={{ minWidth: 220 }}>
                        <InputLabel id="history-type-label">Tipo de tarea</InputLabel>
                        <Select
                            labelId="history-type-label"
                            value={type}
                            label="Tipo de tarea"
                            onChange={changeType}
                        >
                            <MenuItem value="">Todos los tipos</MenuItem>
                            {Object.entries(TYPE_LABELS).map(([value, label]) => (
                                <MenuItem key={value} value={value}>
                                    {label}
                                </MenuItem>
                            ))}
                        </Select>
                    </FormControl>
                    <FormControl size="small" sx={{ minWidth: 190 }}>
                        <InputLabel id="history-state-label">Estado</InputLabel>
                        <Select
                            labelId="history-state-label"
                            value={state}
                            label="Estado"
                            onChange={changeState}
                        >
                            <MenuItem value="">Todos los estados</MenuItem>
                            {Object.entries(STATE_LABELS).map(([value, label]) => (
                                <MenuItem key={value} value={value}>
                                    {label}
                                </MenuItem>
                            ))}
                        </Select>
                    </FormControl>
                    <FormControl size="small" sx={{ minWidth: 190 }}>
                        <InputLabel id="history-outcome-label">Resultado</InputLabel>
                        <Select
                            labelId="history-outcome-label"
                            value={outcome}
                            label="Resultado"
                            onChange={changeOutcome}
                        >
                            <MenuItem value="">Todos los resultados</MenuItem>
                            {Object.entries(OUTCOME_LABELS).map(([value, label]) => (
                                <MenuItem key={value} value={value}>
                                    {label}
                                </MenuItem>
                            ))}
                        </Select>
                    </FormControl>
                    {filtersActive && (
                        <Button
                            onClick={() => {
                                setPage(0);
                                setType("");
                                setState("");
                                setOutcome("");
                            }}
                        >
                            Limpiar filtros
                        </Button>
                    )}
                    <Box sx={{ flexGrow: 1 }} />
                    <Tooltip title="Actualizar historial">
                        <span>
                            <IconButton
                                aria-label="Actualizar historial"
                                onClick={loadHistory}
                                disabled={loading}
                            >
                                <Refresh />
                            </IconButton>
                        </span>
                    </Tooltip>
                </Box>
            </Paper>

            {error && (
                <Alert
                    severity="error"
                    aria-live="assertive"
                    sx={{ mb: 2 }}
                    action={
                        <Button color="inherit" size="small" onClick={loadHistory}>
                            Reintentar
                        </Button>
                    }
                >
                    {error}
                </Alert>
            )}

            <Paper elevation={0} variant="outlined" sx={{ overflow: "hidden" }}>
                {loading && (
                    <Box
                        className="flex items-center justify-center"
                        sx={{ minHeight: 260 }}
                        aria-live="polite"
                    >
                        <CircularProgress size={34} />
                        <Typography sx={{ ml: 2 }}>Cargando tareas reales…</Typography>
                    </Box>
                )}
                {!loading && items.length === 0 && !error && (
                    <Box sx={{ p: 5, textAlign: "center" }}>
                        <Typography variant="h6">No hay tareas para estos filtros</Typography>
                        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
                            Ejecuta una tarea desde Control de simulación o cambia los filtros.
                        </Typography>
                    </Box>
                )}
                {!loading && items.length > 0 && (
                    <TableContainer>
                        <Table aria-label="Historial de tareas ROS">
                            <TableHead>
                                <TableRow>
                                    <TableCell>Tarea</TableCell>
                                    <TableCell>Estado</TableCell>
                                    <TableCell>Resultado</TableCell>
                                    <TableCell>Progreso</TableCell>
                                    <TableCell>Duración</TableCell>
                                    <TableCell>Inicio</TableCell>
                                    <TableCell align="right">Detalle</TableCell>
                                </TableRow>
                            </TableHead>
                            <TableBody>
                                {items.map((task) => (
                                    <TableRow key={task.id} hover>
                                        <TableCell>
                                            <Typography variant="body2" sx={{ fontWeight: 600 }}>
                                                {taskTypeLabel(task.type)}
                                            </Typography>
                                            <Typography variant="caption" color="text.secondary">
                                                Sesión {String(task.sessionId).slice(0, 8)}
                                            </Typography>
                                        </TableCell>
                                        <TableCell>
                                            <Chip
                                                size="small"
                                                color={taskStateColor(task.state)}
                                                label={taskStateLabel(task.state)}
                                            />
                                        </TableCell>
                                        <TableCell>
                                            <Chip
                                                size="small"
                                                variant="outlined"
                                                color={taskOutcomeColor(task.outcomeState)}
                                                label={taskOutcomeLabel(task.outcomeState)}
                                            />
                                        </TableCell>
                                        <TableCell>
                                            {Math.round((Number(task.progress) || 0) * 100)} %
                                        </TableCell>
                                        <TableCell>{formatTaskDuration(task)}</TableCell>
                                        <TableCell>{formatDate(task.createdAt)}</TableCell>
                                        <TableCell align="right">
                                            <Tooltip title="Ver parámetros y resultado">
                                                <IconButton
                                                    aria-label={`Ver detalle de ${taskTypeLabel(task.type)}`}
                                                    onClick={() => setSelectedTask(task)}
                                                >
                                                    <Visibility />
                                                </IconButton>
                                            </Tooltip>
                                        </TableCell>
                                    </TableRow>
                                ))}
                            </TableBody>
                        </Table>
                    </TableContainer>
                )}
                {!loading && !error && total > 0 && (
                    <TablePagination
                        component="div"
                        count={total}
                        page={page}
                        rowsPerPage={rowsPerPage}
                        rowsPerPageOptions={[5, 10, 25, 50]}
                        labelRowsPerPage="Tareas por página"
                        onPageChange={(_, nextPage) => setPage(nextPage)}
                        onRowsPerPageChange={(event) => {
                            setRowsPerPage(Number(event.target.value));
                            setPage(0);
                        }}
                    />
                )}
            </Paper>

            <Dialog open={Boolean(selectedTask)} onClose={() => setSelectedTask(null)} fullWidth maxWidth="md">
                <DialogTitle sx={{ pr: 6 }}>
                    {selectedTask ? taskTypeLabel(selectedTask.type) : "Detalle de tarea"}
                    <IconButton
                        aria-label="Cerrar detalle"
                        onClick={() => setSelectedTask(null)}
                        sx={{ position: "absolute", right: 8, top: 8 }}
                    >
                        <Close />
                    </IconButton>
                </DialogTitle>
                {selectedTask && (
                    <DialogContent dividers>
                        <Box className="flex" sx={{ gap: 1, flexWrap: "wrap", mb: 2 }}>
                            <Chip
                                label={taskStateLabel(selectedTask.state)}
                                color={taskStateColor(selectedTask.state)}
                            />
                            <Chip
                                label={taskOutcomeLabel(selectedTask.outcomeState)}
                                color={taskOutcomeColor(selectedTask.outcomeState)}
                                variant="outlined"
                            />
                            <Chip
                                label={`Progreso ${Math.round((Number(selectedTask.progress) || 0) * 100)} %`}
                            />
                        </Box>
                        {(selectedTask.error || selectedTask.outcomeReason) && (
                            <Alert severity={selectedTask.error ? "error" : "info"} sx={{ mb: 2 }}>
                                {selectedTask.error || selectedTask.outcomeReason}
                            </Alert>
                        )}
                        <Typography variant="subtitle2">Parámetros enviados</Typography>
                        <Box
                            component="pre"
                            sx={{ p: 2, overflow: "auto", bgcolor: "grey.100", borderRadius: 1, fontSize: 13 }}
                        >
                            {formatTaskPayload(selectedTask.parameters)}
                        </Box>
                        <Typography variant="subtitle2" sx={{ mt: 2 }}>
                            Resultado informado por ROS
                        </Typography>
                        <Box
                            component="pre"
                            sx={{ p: 2, overflow: "auto", bgcolor: "grey.100", borderRadius: 1, fontSize: 13 }}
                        >
                            {formatTaskPayload(selectedTask.result)}
                        </Box>
                        <Typography variant="body2" color="text.secondary" sx={{ mt: 2 }}>
                            Sesión: {selectedTask.sessionId} · Inicio:{" "}
                            {formatDate(selectedTask.startedAt || selectedTask.createdAt)} · Fin:{" "}
                            {formatDate(selectedTask.completedAt || selectedTask.updatedAt)}
                        </Typography>
                    </DialogContent>
                )}
                <DialogActions>
                    <Button onClick={() => setSelectedTask(null)}>Cerrar</Button>
                </DialogActions>
            </Dialog>
        </Box>
    );
}

export default TaskRunHistoryApp;
