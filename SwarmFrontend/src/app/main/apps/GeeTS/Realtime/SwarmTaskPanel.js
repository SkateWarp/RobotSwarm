import { useMemo, useState } from "react";
import PropTypes from "prop-types";
import {
    Alert,
    Box,
    Button,
    ButtonBase,
    Chip,
    Divider,
    FormControl,
    Grid,
    InputLabel,
    LinearProgress,
    MenuItem,
    Select,
    TextField,
    Typography,
} from "@mui/material";

const TASK_TYPES = [
    {
        value: "FollowLeader",
        label: "Seguir al líder",
        title: "Seguir al líder",
        description: "Un robot marca la trayectoria y el resto mantiene una separación coordinada.",
    },
    {
        value: "Figure",
        label: "Figura o letra",
        title: "Formar figura",
        description: "Distribuye todo el enjambre en una forma geométrica o una letra.",
    },
    {
        value: "CollaborativeTransport",
        label: "Transporte colaborativo",
        title: "Transportar en equipo",
        description: "Todos buscan la carga; al hallarla se reúnen y la empujan de forma coordinada.",
    },
];

const TERMINAL_TASK_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const MUTED_TRANSPORT_STATES = new Set(["Paused", "Cancelling", "Cancelled", "Failed"]);
const SUPPORTED_LETTERS = "ABCDEFGHIJKLMNOPRSTUVWXYZ".split("");

const TASK_STATE_LABELS = {
    Queued: "En cola",
    Running: "En ejecución",
    Paused: "En pausa",
    Cancelling: "Cancelando",
    Completed: "Completada",
    Cancelled: "Cancelada",
    Failed: "Fallida",
};

const OUTCOME_LABELS = {
    Pending: "Pendiente de verificación",
    Succeeded: "Correcto",
    Failed: "Fallido",
    Cancelled: "Cancelado",
};

const TRANSPORT_PHASES = {
    SEARCH: "Búsqueda",
    APPROACH: "Reagrupación",
    PUSH: "Empuje coordinado",
    DONE: "Entrega completada",
    FAILED: "Transporte fallido",
};

export const validateNumberField = (value, minimum, maximum, label) => {
    if (
        value === "" ||
        value === null ||
        value === undefined ||
        (typeof value === "string" && value.trim() === "")
    ) {
        return `${label}: introduce un valor.`;
    }

    const parsed = Number(value);
    if (!Number.isFinite(parsed)) {
        return `${label}: el valor debe ser numérico.`;
    }
    if (parsed < minimum || parsed > maximum) {
        return `${label}: usa un valor entre ${minimum} y ${maximum}.`;
    }
    return "";
};

const taskColor = (state) => {
    if (state === "Running" || state === "Completed") return "success";
    if (state === "Paused" || state === "Cancelling") return "warning";
    if (state === "Failed") return "error";
    return "info";
};

const finiteInteger = (value) => (Number.isInteger(value) && value >= 0 ? value : null);

const coordinate = (value) => {
    if (value === null || value === undefined || value === "") return null;
    const number = Number(value);
    return Number.isFinite(number) ? number.toFixed(2) : null;
};

export const describeTransportResult = (task) => {
    // Keep the persisted result available for history without implying that
    // robots are still moving after the task pauses, cancels or fails.
    if (MUTED_TRANSPORT_STATES.has(task?.state)) return null;

    const transport = task?.result?.transport;
    if (!transport || typeof transport !== "object") return null;

    const phase = typeof transport.phase === "string" ? transport.phase.toUpperCase() : "";
    const phaseLabel = TRANSPORT_PHASES[phase] || transport.phase || "En curso";
    const searching = finiteInteger(transport.searching_robot_count);
    const contributors = finiteInteger(transport.useful_contributor_count);
    const { discovery } = transport;
    const notified = Array.isArray(discovery?.notified_robots) ? discovery.notified_robots.length : null;
    const finder = typeof discovery?.finder === "string" ? discovery.finder : "";
    const positionX = coordinate(discovery?.object_position?.x);
    const positionY = coordinate(discovery?.object_position?.y);
    const position = positionX !== null && positionY !== null ? ` (${positionX}, ${positionY})` : "";

    let summary = "El worker está actualizando el estado del transporte.";
    let severity = "info";
    if (phase === "SEARCH") {
        summary =
            searching === null
                ? "El enjambre está recorriendo el entorno para localizar la carga."
                : `${searching} ${
                      searching === 1 ? "robot está buscando" : "robots están buscando"
                  } la carga.`;
    } else if (phase === "APPROACH") {
        summary = finder
            ? `${finder} localizó la carga${position}${
                  notified === null ? "" : ` y avisó a ${notified} compañeros`
              }. El enjambre se está reuniendo.`
            : "Los robots se están reuniendo alrededor de la carga.";
        severity = "success";
    } else if (phase === "PUSH") {
        summary = transport.all_pushers_confirmed
            ? `El empuje conjunto está confirmado${
                  contributors === null ? "" : ` para ${contributors} robots`
              }.`
            : "El enjambre se está posicionando para iniciar el empuje conjunto.";
        severity = "success";
    } else if (phase === "DONE") {
        summary = transport.all_pushers_confirmed
            ? `La carga llegó al objetivo con ${contributors ?? "todos los"} robots como participantes útiles.`
            : "La carga llegó al objetivo.";
        severity = "success";
    } else if (phase === "FAILED") {
        summary = "El transporte no pudo completarse. Revisa el error de la tarea antes de reintentar.";
        severity = "error";
    }

    return {
        phase,
        phaseLabel,
        summary,
        severity,
        searching,
        contributors,
        allPushersConfirmed: transport.all_pushers_confirmed === true,
        finder,
        position,
        notified,
    };
};

const fieldProps = (error) => ({
    error: Boolean(error),
    helperText: error || " ",
});

function SwarmTaskPanel({ session, tasks, busy, onStart, onTaskAction }) {
    const [taskType, setTaskType] = useState("FollowLeader");
    const [leaderMode, setLeaderMode] = useState("circular");
    const [followDistance, setFollowDistance] = useState(0.7);
    const [pathRadius, setPathRadius] = useState(2);
    const [formationType, setFormationType] = useState("circle");
    const [formationLetter, setFormationLetter] = useState("A");
    const [formationSpacing, setFormationSpacing] = useState(0.7);
    const [targetX, setTargetX] = useState(3);
    const [targetY, setTargetY] = useState(3);

    const activeTask = tasks.find((task) => !TERMINAL_TASK_STATES.has(task.state));
    const latestTask = activeTask || tasks[0];
    const canControl = ["Ready", "Active", "Paused"].includes(session.state);
    const canStart = canControl && !session.isEmergencyStopped && !activeTask;
    const latestProgress = Math.min(1, Math.max(0, Number(latestTask?.progress) || 0));
    const verifiedOutcome = latestTask?.outcomeState;
    const selectedType = TASK_TYPES.find((item) => item.value === taskType);
    const latestType = TASK_TYPES.find((item) => item.value === latestTask?.type);
    const transportResult = describeTransportResult(latestTask);

    const validation = useMemo(() => {
        const errors = {};
        if (taskType === "FollowLeader") {
            errors.followDistance = validateNumberField(followDistance, 0.35, 2, "Separación entre robots");
            errors.pathRadius = validateNumberField(pathRadius, 0.5, 4, "Radio de trayectoria");
        } else if (taskType === "Figure") {
            errors.formationSpacing = validateNumberField(
                formationSpacing,
                0.35,
                2,
                "Separación de la figura"
            );
        } else {
            errors.targetX = validateNumberField(targetX, -4, 4, "Objetivo X");
            errors.targetY = validateNumberField(targetY, -4, 4, "Objetivo Y");
        }
        return errors;
    }, [followDistance, formationSpacing, pathRadius, targetX, targetY, taskType]);

    const validationErrors = Object.values(validation).filter(Boolean);

    const buildParameters = () => {
        if (taskType === "FollowLeader") {
            return {
                leader_mode: leaderMode,
                config: {
                    leader_mode: leaderMode,
                    follow_distance: Number(followDistance),
                    radius: Number(pathRadius),
                },
            };
        }

        if (taskType === "Figure") {
            const shape = formationType === "letter" ? formationLetter.toUpperCase() : formationType;
            return {
                formation_type: shape,
                movement_mode: "static",
                config: {
                    formation_type: shape,
                    movement_mode: "static",
                    spacing: Number(formationSpacing),
                },
            };
        }

        return {
            target_x: Number(targetX),
            target_y: Number(targetY),
            config: {
                target_x: Number(targetX),
                target_y: Number(targetY),
                transport_planner: "grf",
            },
        };
    };

    const startTask = () => {
        if (validationErrors.length > 0) return;
        onStart(taskType, buildParameters());
    };

    return (
        <Box data-testid="task-panel">
            <Box className="flex items-start justify-between" sx={{ mb: 2, gap: 2 }}>
                <Box>
                    <Typography variant="h6">Tarea del enjambre</Typography>
                    <Typography variant="body2" color="text.secondary">
                        3 · Ejecuta una tarea con parámetros verificados
                    </Typography>
                </Box>
                {latestTask && (
                    <Box sx={{ textAlign: "right" }}>
                        <Chip
                            data-testid="task-state"
                            label={TASK_STATE_LABELS[latestTask.state] || latestTask.state}
                            color={taskColor(latestTask.state)}
                            size="small"
                        />
                        <Typography
                            variant="caption"
                            color="text.secondary"
                            sx={{ display: "block", mt: 0.5 }}
                        >
                            Estado informado por el worker
                        </Typography>
                    </Box>
                )}
            </Box>

            {!canControl && (
                <Alert severity="info" sx={{ mb: 2 }} aria-live="polite">
                    Los controles se habilitarán cuando el worker GPU confirme que la sesión está lista.
                </Alert>
            )}

            {session.isEmergencyStopped && (
                <Alert severity="error" sx={{ mb: 2 }} aria-live="assertive">
                    La parada de emergencia está activa. Restablécela antes de iniciar o reanudar una tarea.
                </Alert>
            )}

            {latestTask && (
                <Box data-testid="task-result" aria-live="polite" sx={{ mb: 3 }}>
                    <Box className="flex items-center justify-between" sx={{ mb: 1 }}>
                        <Box>
                            <Typography variant="body2">{latestType?.label || latestTask.type}</Typography>
                            {latestType && (
                                <Typography variant="caption" color="text.secondary">
                                    {latestType.title}
                                </Typography>
                            )}
                        </Box>
                        <Typography variant="body2" color="text.secondary">
                            {Math.round(latestProgress * 100)}%
                        </Typography>
                    </Box>
                    <LinearProgress
                        aria-label="Progreso de la tarea"
                        variant="determinate"
                        value={latestProgress * 100}
                    />
                    {verifiedOutcome && verifiedOutcome !== "Pending" && (
                        <Box sx={{ mt: 1 }}>
                            <Typography variant="body2">
                                Resultado verificado: {OUTCOME_LABELS[verifiedOutcome] || verifiedOutcome}
                            </Typography>
                        </Box>
                    )}
                    {latestTask.error && (
                        <Alert severity="error" sx={{ mt: 2 }}>
                            {latestTask.error}
                        </Alert>
                    )}
                    {transportResult && (
                        <Alert
                            data-testid="transport-phase"
                            severity={transportResult.severity}
                            sx={{ mt: 2 }}
                        >
                            <Typography variant="subtitle2">Fase: {transportResult.phaseLabel}</Typography>
                            <Typography variant="body2">{transportResult.summary}</Typography>
                            {(transportResult.searching !== null ||
                                transportResult.contributors !== null ||
                                transportResult.notified !== null) && (
                                <Typography variant="caption" sx={{ display: "block", mt: 0.75 }}>
                                    {[
                                        transportResult.searching !== null
                                            ? `buscando: ${transportResult.searching}`
                                            : null,
                                        transportResult.finder
                                            ? `hallazgo: ${transportResult.finder}${transportResult.position}`
                                            : null,
                                        transportResult.notified !== null
                                            ? `avisados: ${transportResult.notified}`
                                            : null,
                                        transportResult.contributors !== null
                                            ? `participantes útiles: ${transportResult.contributors}`
                                            : null,
                                    ]
                                        .filter(Boolean)
                                        .join(" · ")}
                                </Typography>
                            )}
                        </Alert>
                    )}
                </Box>
            )}

            {activeTask ? (
                <Grid container spacing={1.5}>
                    {activeTask.state === "Running" && (
                        <Grid item xs={12} sm={6}>
                            <Button
                                fullWidth
                                variant="outlined"
                                onClick={() => onTaskAction("pause", activeTask.id)}
                                disabled={busy}
                            >
                                Pausar
                            </Button>
                        </Grid>
                    )}
                    {activeTask.state === "Paused" && (
                        <Grid item xs={12} sm={6}>
                            <Button
                                fullWidth
                                variant="contained"
                                onClick={() => onTaskAction("resume", activeTask.id)}
                                disabled={busy || session.isEmergencyStopped}
                            >
                                Reanudar
                            </Button>
                        </Grid>
                    )}
                    <Grid item xs={12} sm={6}>
                        <Button
                            fullWidth
                            color="error"
                            variant="outlined"
                            onClick={() => onTaskAction("cancel", activeTask.id)}
                            disabled={busy || activeTask.state === "Cancelling"}
                        >
                            {activeTask.state === "Cancelling" ? "Cancelando…" : "Cancelar tarea"}
                        </Button>
                    </Grid>
                </Grid>
            ) : (
                <>
                    <Grid container spacing={1.5} role="radiogroup" aria-label="Tipo de tarea">
                        {TASK_TYPES.map((item) => {
                            const selected = item.value === taskType;
                            return (
                                <Grid item xs={12} md={4} key={item.value}>
                                    <ButtonBase
                                        data-testid={`task-option-${item.value}`}
                                        role="radio"
                                        aria-checked={selected}
                                        onClick={() => setTaskType(item.value)}
                                        sx={{
                                            width: "100%",
                                            height: "100%",
                                            p: 1.5,
                                            alignItems: "flex-start",
                                            justifyContent: "flex-start",
                                            textAlign: "left",
                                            border: 2,
                                            borderColor: selected ? "primary.main" : "divider",
                                            borderRadius: 2,
                                            bgcolor: selected ? "action.selected" : "background.paper",
                                            transition: "border-color 120ms ease, background-color 120ms ease",
                                        }}
                                    >
                                        <Box>
                                            <Typography
                                                variant="subtitle2"
                                                color={selected ? "primary.main" : "text.primary"}
                                            >
                                                {item.title}
                                            </Typography>
                                            <Typography variant="caption" color="text.secondary">
                                                {item.description}
                                            </Typography>
                                        </Box>
                                    </ButtonBase>
                                </Grid>
                            );
                        })}
                    </Grid>

                    <Divider sx={{ my: 2 }} />

                    <FormControl fullWidth size="small" sx={{ mb: 2 }}>
                        <InputLabel id="task-type-label">Tarea</InputLabel>
                        <Select
                            labelId="task-type-label"
                            value={taskType}
                            label="Tarea"
                            onChange={(event) => setTaskType(event.target.value)}
                        >
                            {TASK_TYPES.map((item) => (
                                <MenuItem key={item.value} value={item.value}>
                                    {item.label}
                                </MenuItem>
                            ))}
                        </Select>
                        <Typography variant="caption" color="text.secondary" sx={{ mt: 0.75 }}>
                            Tipo seleccionado: {selectedType?.title}
                        </Typography>
                    </FormControl>

                    {taskType === "FollowLeader" && (
                        <Grid container spacing={2}>
                            <Grid item xs={12}>
                                <FormControl fullWidth size="small">
                                    <InputLabel id="leader-mode-label">Trayectoria del líder</InputLabel>
                                    <Select
                                        labelId="leader-mode-label"
                                        value={leaderMode}
                                        label="Trayectoria del líder"
                                        onChange={(event) => setLeaderMode(event.target.value)}
                                    >
                                        <MenuItem value="circular">Circular</MenuItem>
                                        <MenuItem value="square">Cuadrada</MenuItem>
                                        <MenuItem value="figure8">Ocho</MenuItem>
                                    </Select>
                                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.75 }}>
                                        Trayectoria que seguirá el robot líder.
                                    </Typography>
                                </FormControl>
                            </Grid>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Separación de seguidores (m)"
                                    value={followDistance}
                                    inputProps={{ min: 0.35, max: 2, step: 0.05 }}
                                    onChange={(event) => setFollowDistance(event.target.value)}
                                    {...fieldProps(validation.followDistance)}
                                />
                            </Grid>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Radio de trayectoria (m)"
                                    value={pathRadius}
                                    inputProps={{ min: 0.5, max: 4, step: 0.25 }}
                                    onChange={(event) => setPathRadius(event.target.value)}
                                    {...fieldProps(validation.pathRadius)}
                                />
                            </Grid>
                        </Grid>
                    )}

                    {taskType === "Figure" && (
                        <Grid container spacing={2}>
                            <Grid item xs={12} sm={6}>
                                <FormControl fullWidth size="small">
                                    <InputLabel id="formation-type-label">Figura</InputLabel>
                                    <Select
                                        labelId="formation-type-label"
                                        value={formationType}
                                        label="Figura"
                                        onChange={(event) => setFormationType(event.target.value)}
                                    >
                                        <MenuItem value="circle">Círculo</MenuItem>
                                        <MenuItem value="square">Cuadrado</MenuItem>
                                        <MenuItem value="triangle">Triángulo</MenuItem>
                                        <MenuItem value="diamond">Rombo</MenuItem>
                                        <MenuItem value="line">Línea</MenuItem>
                                        <MenuItem value="v_formation">Formación en V</MenuItem>
                                        <MenuItem value="letter">Letra</MenuItem>
                                    </Select>
                                </FormControl>
                            </Grid>
                            {formationType === "letter" && (
                                <Grid item xs={12} sm={6}>
                                    <FormControl fullWidth size="small">
                                        <InputLabel id="formation-letter-label">Letra</InputLabel>
                                        <Select
                                            labelId="formation-letter-label"
                                            value={formationLetter}
                                            label="Letra"
                                            onChange={(event) => setFormationLetter(event.target.value)}
                                        >
                                            {SUPPORTED_LETTERS.map((letter) => (
                                                <MenuItem key={letter} value={letter}>
                                                    {letter}
                                                </MenuItem>
                                            ))}
                                        </Select>
                                    </FormControl>
                                </Grid>
                            )}
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    label="Movimiento"
                                    value="Estático"
                                    disabled
                                />
                            </Grid>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Separación de robots (m)"
                                    value={formationSpacing}
                                    inputProps={{ min: 0.35, max: 2, step: 0.05 }}
                                    onChange={(event) => setFormationSpacing(event.target.value)}
                                    {...fieldProps(validation.formationSpacing)}
                                />
                            </Grid>
                        </Grid>
                    )}

                    {taskType === "CollaborativeTransport" && (
                        <Grid container spacing={2}>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Target X"
                                    value={targetX}
                                    inputProps={{ min: -4, max: 4, step: 0.25 }}
                                    onChange={(event) => setTargetX(event.target.value)}
                                    {...fieldProps(validation.targetX)}
                                />
                            </Grid>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Target Y"
                                    value={targetY}
                                    inputProps={{ min: -4, max: 4, step: 0.25 }}
                                    onChange={(event) => setTargetY(event.target.value)}
                                    {...fieldProps(validation.targetY)}
                                />
                            </Grid>
                            <Grid item xs={12}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    label="Planner"
                                    value="Coordinated GRF"
                                    disabled
                                    helperText="La flota busca, comunica el hallazgo y converge antes del empuje."
                                />
                            </Grid>
                        </Grid>
                    )}

                    {validationErrors.length > 0 && (
                        <Alert
                            data-testid="task-validation"
                            severity="warning"
                            aria-live="assertive"
                            sx={{ mt: 1 }}
                        >
                            Revisa los valores marcados antes de iniciar la tarea.
                        </Alert>
                    )}

                    <Button
                        fullWidth
                        variant="contained"
                        sx={{ mt: 2 }}
                        onClick={startTask}
                        disabled={busy || !canStart || validationErrors.length > 0}
                    >
                        {busy ? "Enviando…" : "Iniciar tarea"}
                    </Button>
                    <Typography variant="caption" color="text.secondary" sx={{ display: "block", mt: 1 }}>
                        La orden se enviará a todos los robots activos de esta sesión.
                    </Typography>
                </>
            )}
        </Box>
    );
}

SwarmTaskPanel.propTypes = {
    session: PropTypes.object.isRequired,
    tasks: PropTypes.array.isRequired,
    busy: PropTypes.bool.isRequired,
    onStart: PropTypes.func.isRequired,
    onTaskAction: PropTypes.func.isRequired,
};

export default SwarmTaskPanel;
