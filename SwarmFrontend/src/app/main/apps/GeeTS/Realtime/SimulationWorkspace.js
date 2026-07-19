import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    Box,
    Button,
    Chip,
    CircularProgress,
    Divider,
    FormControl,
    Grid,
    InputLabel,
    MenuItem,
    Paper,
    Select,
    Slider,
    Typography,
} from "@mui/material";
import SimulationSessionService from "../../../../../services/SimulationSessionService";
import HlsViewer from "./HlsViewer";
import SwarmTaskPanel from "./SwarmTaskPanel";
import WhepViewer from "./WhepViewer";

const TERMINAL_STATES = new Set(["Stopped", "Failed", "Expired"]);
const TERMINAL_TASK_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const TERMINAL_VIEWER_COMMAND_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const UNAVAILABLE_ROBOT_STATES = new Set(["Removed", "Failed"]);
const CLEANUP_STATES = new Set(["Failed", "Expired"]);

const commandTimestamp = (command) => {
    const value = Date.parse(command?.updatedAt || command?.createdAt);
    return Number.isFinite(value) ? value : null;
};

const mergeViewerCommand = (current, incoming) => {
    if (!incoming) return current;
    if (current?.id && incoming.id && current.id.toLowerCase() !== incoming.id.toLowerCase()) {
        return current;
    }

    const currentTime = commandTimestamp(current);
    const incomingTime = commandTimestamp(incoming);
    if (currentTime !== null && incomingTime !== null && incomingTime < currentTime) {
        return current;
    }
    if (
        TERMINAL_VIEWER_COMMAND_STATES.has(current?.state) &&
        !TERMINAL_VIEWER_COMMAND_STATES.has(incoming.state)
    ) {
        return current;
    }

    return { ...current, ...incoming };
};

export const mergeViewerCommandEvent = (lease, event) => {
    const commandId = lease?.command?.id;
    if (
        !commandId ||
        event?.eventType !== "CommandUpdated" ||
        typeof event.commandId !== "string" ||
        commandId.toLowerCase() !== event.commandId.toLowerCase()
    ) {
        return lease;
    }

    const command = mergeViewerCommand(lease.command, {
        id: event.commandId,
        state: event.state,
        error: event.error || null,
        updatedAt: event.timestamp,
    });
    return {
        ...lease,
        isReady: command.state === "Completed",
        command,
    };
};

export const mergeViewerLeaseStatus = (lease, status) => {
    if (!lease?.leaseId || !status?.leaseId || lease.leaseId.toLowerCase() !== status.leaseId.toLowerCase()) {
        return lease;
    }

    const command = mergeViewerCommand(lease.command, status.command);
    return {
        ...lease,
        expiresAt: status.expiresAt || lease.expiresAt,
        revokedAt: status.revokedAt,
        isReady: command?.state === "Completed" || Boolean(status.isReady),
        command,
    };
};

export const viewerCommandFailureMessage = (command) => {
    if (command?.error) return `El worker no pudo preparar el visor: ${command.error}`;
    if (command?.state === "Cancelled") {
        return "La preparación del visor fue cancelada. Puedes solicitar una vista nueva.";
    }
    return "El worker no pudo preparar el visor. Puedes volver a intentarlo sin crear otra sesión.";
};

export const sendViewerControlInput = (connection, sessionId, leaseId, input) =>
    connection.invoke("SendViewerInput", sessionId, leaseId, input);

const SESSION_STATE_LABELS = {
    Queued: "En cola",
    Provisioning: "Preparando recursos",
    Ready: "Lista para operar",
    Active: "Operando",
    Paused: "En pausa",
    Stopping: "Deteniendo y limpiando",
    Stopped: "Detenida",
    Failed: "Fallida",
    Expired: "Expirada",
};

const stateColor = (state) => {
    if (state === "Ready" || state === "Active") return "success";
    if (state === "Failed" || state === "Expired") return "error";
    if (state === "Stopping" || state === "Paused") return "warning";
    return "info";
};

const validationMessages = (errors) => {
    if (!errors || typeof errors !== "object") return [];
    return Object.values(errors).flatMap((value) => {
        if (Array.isArray(value)) return value.filter((item) => typeof item === "string");
        return typeof value === "string" ? [value] : [];
    });
};

export const requestMessage = (requestError, fallback) => {
    const response = requestError?.response?.data;
    if (typeof response === "string" && response.trim()) return response;

    const validationMessage = validationMessages(response?.errors)[0];
    return (
        response?.message ||
        response?.detail ||
        validationMessage ||
        response?.title ||
        requestError?.response?.statusText ||
        fallback
    );
};

const viewerPlaceholder = (lease, session, cleanupPending) => {
    if (lease) {
        return {
            title: "El acceso privado está activo",
            body: "La sesión todavía no informó una ruta de video compatible.",
        };
    }
    if (session) {
        return {
            title: "Abre el visor de tu sesión",
            body: "El acceso es temporal, pertenece a esta cuenta y no comparte la pantalla VNC del host.",
        };
    }
    if (cleanupPending) {
        return {
            title: "Finaliza la limpieza anterior",
            body: "El visor estará disponible cuando el worker GPU confirme que liberó los recursos.",
        };
    }
    return {
        title: "Primero crea una sesión",
        body: "Cada usuario recibe una simulación y un visor privados.",
    };
};

function SimulationWorkspace() {
    const [sessions, setSessions] = useState([]);
    const [robots, setRobots] = useState([]);
    const [tasks, setTasks] = useState([]);
    const [robotCount, setRobotCount] = useState(10);
    const [fleetCount, setFleetCount] = useState(10);
    const [maxRobotCount, setMaxRobotCount] = useState(10);
    const [viewerSource, setViewerSource] = useState("Scene");
    const [viewerRobotId, setViewerRobotId] = useState("");
    const [viewerLease, setViewerLease] = useState(null);
    const [hlsUnavailable, setHlsUnavailable] = useState(false);
    const [viewerControlReady, setViewerControlReady] = useState(false);
    const [connectionVersion, setConnectionVersion] = useState(0);
    const [loading, setLoading] = useState(true);
    const [submitting, setSubmitting] = useState(false);
    const [error, setError] = useState("");
    const connectionRef = useRef(null);
    const joinedSessionRef = useRef(null);

    const activeSession = useMemo(
        () => sessions.find((session) => !TERMINAL_STATES.has(session.state)),
        [sessions]
    );
    const cleanupSession = useMemo(
        () =>
            sessions.find((session) => CLEANUP_STATES.has(session.state) && Boolean(session.computeWorkerId)),
        [sessions]
    );
    const displayedSession = activeSession || cleanupSession;
    const activeSessionId = activeSession?.id;
    const activeRobotCount = activeSession?.desiredRobotCount;
    const availableRobots = useMemo(
        () => robots.filter((robot) => !UNAVAILABLE_ROBOT_STATES.has(robot.state)),
        [robots]
    );
    const canControl = Boolean(activeSession && ["Ready", "Active", "Paused"].includes(activeSession.state));
    const canResizeFleet = Boolean(
        activeSession?.state === "Ready" && !tasks.some((task) => !TERMINAL_TASK_STATES.has(task.state))
    );
    const viewerCommandState = viewerLease?.command?.state;
    const viewerCommandPending = Boolean(
        viewerLease?.command && !TERMINAL_VIEWER_COMMAND_STATES.has(viewerCommandState)
    );
    const viewerReady = viewerCommandState === "Completed";
    const viewerPrompt = viewerPlaceholder(viewerLease, activeSession, Boolean(cleanupSession));
    const useWhepFallback = useCallback(() => setHlsUnavailable(true), []);

    const refreshSessions = useCallback(async () => {
        try {
            const result = await SimulationSessionService.list();
            setSessions(result);
            setError("");
        } catch (requestError) {
            setError(requestMessage(requestError, "No fue posible contactar el servicio de simulación."));
        } finally {
            setLoading(false);
        }
    }, []);

    const refreshSessionDetails = useCallback(async (sessionId) => {
        try {
            const [robotList, taskList] = await Promise.all([
                SimulationSessionService.listRobots(sessionId),
                SimulationSessionService.listTasks(sessionId),
            ]);
            setRobots(robotList);
            setTasks(taskList);
        } catch (requestError) {
            setError(requestMessage(requestError, "No fue posible actualizar los detalles de la sesión."));
        }
    }, []);

    useEffect(() => {
        refreshSessions();
        const interval = window.setInterval(refreshSessions, 5000);
        return () => window.clearInterval(interval);
    }, [refreshSessions]);

    useEffect(() => {
        SimulationSessionService.getLimits()
            .then((limits) => {
                const maximum = Math.max(1, Number(limits.maxRobotsPerSession) || 10);
                setMaxRobotCount(maximum);
                setRobotCount((current) => Math.min(current, maximum));
                setFleetCount((current) => Math.min(current, maximum));
            })
            .catch(() => {
                // The session request below will show the useful connection error.
            });
    }, []);

    useEffect(() => {
        if (!activeSessionId) {
            setRobots([]);
            setTasks([]);
            setViewerLease(null);
            setViewerControlReady(false);
            return undefined;
        }

        setFleetCount(activeRobotCount);
        setViewerLease(null);
        refreshSessionDetails(activeSessionId);
        const interval = window.setInterval(() => refreshSessionDetails(activeSessionId), 3000);
        return () => window.clearInterval(interval);
    }, [activeRobotCount, activeSessionId, refreshSessionDetails]);

    useEffect(() => {
        if (!availableRobots.some((robot) => robot.runtimeId === viewerRobotId)) {
            setViewerRobotId(availableRobots[0]?.runtimeId || "");
            if (viewerSource === "RobotCamera") {
                setViewerLease(null);
            }
        }
    }, [availableRobots, viewerRobotId, viewerSource]);

    useEffect(() => {
        if (!canControl) {
            setViewerLease(null);
        }
    }, [canControl]);

    useEffect(() => {
        setHlsUnavailable(false);
        setViewerControlReady(false);
    }, [viewerLease?.token]);

    useEffect(() => {
        if (!activeSessionId) return undefined;

        let disposed = false;
        const connection = SimulationSessionService.createRealtimeConnection();
        connectionRef.current = connection;

        const refresh = () => {
            refreshSessions();
            refreshSessionDetails(activeSessionId);
        };
        const joinSession = async () => {
            if (disposed) return;
            joinedSessionRef.current = activeSessionId;
            await connection.invoke("JoinSession", activeSessionId);
        };

        connection.on("SessionUpdated", refresh);
        connection.on("TaskUpdated", refresh);
        connection.on("SessionEvent", (event) => {
            setViewerLease((current) => mergeViewerCommandEvent(current, event));
            if (event?.error) {
                setError(
                    typeof event.error === "string"
                        ? event.error
                        : requestMessage({ response: { data: event.error } }, "El worker informó un error.")
                );
            }
            refresh();
        });
        connection.onreconnected(() => {
            setViewerControlReady(false);
            joinSession()
                .then(() => setConnectionVersion((current) => current + 1))
                .catch(() => {});
        });
        connection.onreconnecting(() => setViewerControlReady(false));

        connection
            .start()
            .then(joinSession)
            .then(() => setConnectionVersion((current) => current + 1))
            .catch(() => {
                // Polling above remains available while SignalR reconnects.
            });

        return () => {
            disposed = true;
            const joinedSessionId = joinedSessionRef.current;
            if (joinedSessionId && connection.state === "Connected") {
                connection.invoke("LeaveSession", joinedSessionId).catch(() => {});
            }
            connection.stop().catch(() => {});
            connectionRef.current = null;
            joinedSessionRef.current = null;
        };
    }, [activeSessionId, refreshSessionDetails, refreshSessions]);

    useEffect(() => {
        if (!activeSessionId || !viewerLease?.leaseId || !viewerReady) {
            setViewerControlReady(false);
            return undefined;
        }

        let disposed = false;
        let retryTimer;
        const authorize = async () => {
            const connection = connectionRef.current;
            if (!connection || connection.state !== "Connected") {
                retryTimer = window.setTimeout(authorize, 1500);
                return;
            }
            try {
                const authorization = await connection.invoke(
                    "BeginViewerControl",
                    activeSessionId,
                    viewerLease.leaseId
                );
                if (!disposed) {
                    setViewerControlReady(true);
                    const expiresAt = Date.parse(authorization?.authorizedUntil);
                    const renewIn = Number.isFinite(expiresAt)
                        ? Math.max(1000, Math.min(25000, expiresAt - Date.now() - 5000))
                        : 20000;
                    retryTimer = window.setTimeout(authorize, renewIn);
                }
            } catch (_requestError) {
                if (!disposed) {
                    setViewerControlReady(false);
                    retryTimer = window.setTimeout(authorize, 3000);
                }
            }
        };

        authorize();
        return () => {
            disposed = true;
            window.clearTimeout(retryTimer);
            setViewerControlReady(false);
        };
    }, [activeSessionId, connectionVersion, viewerLease?.leaseId, viewerReady]);

    const sendViewerInput = useCallback(
        (input) => {
            const connection = connectionRef.current;
            if (
                !viewerControlReady ||
                !activeSessionId ||
                !viewerLease?.leaseId ||
                !connection ||
                connection.state !== "Connected"
            ) {
                return Promise.reject(new Error("Viewer control is not connected."));
            }
            // The Hub response lets the viewer stop safely if an input event is rejected.
            return sendViewerControlInput(connection, activeSessionId, viewerLease.leaseId, input);
        },
        [activeSessionId, viewerControlReady, viewerLease?.leaseId]
    );

    useEffect(() => {
        if (!activeSessionId || !viewerLease?.leaseId || !viewerCommandPending) {
            return undefined;
        }

        let disposed = false;
        const refreshViewerStatus = async () => {
            try {
                const status = await SimulationSessionService.getViewerLeaseStatus(
                    activeSessionId,
                    viewerLease.leaseId
                );
                if (!disposed) {
                    setViewerLease((current) => mergeViewerLeaseStatus(current, status));
                }
            } catch (_requestError) {
                // SignalR remains the primary path; the next poll can recover a missed event.
            }
        };

        refreshViewerStatus();
        const interval = window.setInterval(refreshViewerStatus, 1500);
        return () => {
            disposed = true;
            window.clearInterval(interval);
        };
    }, [activeSessionId, viewerCommandPending, viewerLease?.leaseId]);

    const runRequest = async (request, fallbackMessage) => {
        setSubmitting(true);
        setError("");
        try {
            await request();
            await refreshSessions();
            if (activeSession) {
                await refreshSessionDetails(activeSession.id);
            }
        } catch (requestError) {
            setError(requestMessage(requestError, fallbackMessage));
        } finally {
            setSubmitting(false);
        }
    };

    const createSession = () =>
        runRequest(
            () => SimulationSessionService.create(robotCount),
            "No fue posible crear la sesión de simulación."
        );

    const stopSession = () => {
        if (!activeSession) return;
        setViewerLease(null);
        runRequest(
            () => SimulationSessionService.stop(activeSession.id),
            "No fue posible detener la sesión de simulación."
        );
    };

    const retryCleanup = () => {
        if (!cleanupSession) return;
        setViewerLease(null);
        runRequest(
            () => SimulationSessionService.stop(cleanupSession.id),
            "No fue posible reintentar la limpieza de recursos GPU."
        );
    };

    const updateFleet = () => {
        if (!activeSession) return;
        runRequest(
            () => SimulationSessionService.updateFleet(activeSession.id, fleetCount),
            "No fue posible actualizar el tamaño de la flota."
        );
    };

    const startTask = (type, parameters) => {
        if (!activeSession) return;
        runRequest(
            () => SimulationSessionService.startTask(activeSession.id, type, parameters),
            "No fue posible iniciar la tarea del enjambre."
        );
    };

    const changeTask = (action, taskId) => {
        if (!activeSession) return;
        const actions = {
            pause: SimulationSessionService.pauseTask,
            resume: SimulationSessionService.resumeTask,
            cancel: SimulationSessionService.cancelTask,
        };
        if (!actions[action]) return;
        runRequest(
            () => actions[action](activeSession.id, taskId),
            "No fue posible enviar el comando de la tarea."
        );
    };

    const changeEmergencyStop = () => {
        if (!activeSession) return;
        const request = activeSession.isEmergencyStopped
            ? SimulationSessionService.resetEmergencyStop
            : SimulationSessionService.emergencyStop;
        runRequest(
            () => request(activeSession.id),
            "No fue posible enviar el comando de parada de emergencia."
        );
    };

    const openViewer = async () => {
        if (!activeSession || (viewerSource === "RobotCamera" && !viewerRobotId)) {
            return;
        }
        setSubmitting(true);
        setError("");
        try {
            const lease = await SimulationSessionService.createViewerLease(
                activeSession.id,
                viewerSource,
                viewerSource === "RobotCamera" ? viewerRobotId : null
            );
            setHlsUnavailable(false);
            setViewerLease(lease);
        } catch (requestError) {
            setError(requestMessage(requestError, "No fue posible abrir el visor privado."));
        } finally {
            setSubmitting(false);
        }
    };

    let workflowStage = 3;
    if (!activeSession) workflowStage = 1;
    else if (!viewerReady) workflowStage = 2;
    const workflowSteps = [
        { number: 1, label: "Crear sesión", completed: Boolean(activeSession) },
        {
            number: 2,
            label: "Abrir visor privado",
            completed: viewerReady,
        },
        { number: 3, label: "Ejecutar tarea", completed: tasks.length > 0 },
    ];
    let operationalLabel = "Sin sesión activa";
    let operationalColor = "default";
    if (activeSession) {
        operationalLabel = SESSION_STATE_LABELS[activeSession.state] || activeSession.state;
        operationalColor = stateColor(activeSession.state);
    }
    if (cleanupSession) {
        operationalLabel = "Limpieza pendiente";
        operationalColor = "warning";
    }

    let viewerContent = (
        <Box sx={{ px: 4 }}>
            <Typography variant="h6" sx={{ mb: 1 }}>
                {viewerPrompt.title}
            </Typography>
            <Typography variant="body2">{viewerPrompt.body}</Typography>
        </Box>
    );
    if (viewerCommandPending) {
        viewerContent = (
            <Box data-testid="viewer-preparing" sx={{ px: 4 }} aria-live="polite">
                <CircularProgress size={34} color="inherit" sx={{ mb: 2 }} />
                <Typography variant="h6" sx={{ mb: 1 }}>
                    Preparando visor privado…
                </Typography>
                <Typography variant="body2">
                    El worker GPU está iniciando la vista. Esta operación puede tardar unos segundos.
                </Typography>
            </Box>
        );
    } else if (viewerLease?.command && ["Failed", "Cancelled"].includes(viewerCommandState)) {
        viewerContent = (
            <Box data-testid="viewer-command-error" sx={{ width: "100%", px: 3 }}>
                <Alert severity="error" aria-live="assertive">
                    {viewerCommandFailureMessage(viewerLease.command)}
                </Alert>
            </Box>
        );
    } else if (viewerReady && viewerLease?.hlsUrl && viewerLease.token && !hlsUnavailable) {
        viewerContent = (
            <HlsViewer
                url={viewerLease.hlsUrl}
                token={viewerLease.token}
                expiresAt={viewerLease.expiresAt}
                onUnavailable={viewerLease.signalingUrl ? useWhepFallback : undefined}
                interactiveAvailable={viewerControlReady}
                onInput={sendViewerInput}
            />
        );
    } else if (viewerReady && viewerLease?.signalingUrl && viewerLease.token) {
        viewerContent = (
            <WhepViewer
                url={viewerLease.signalingUrl}
                token={viewerLease.token}
                expiresAt={viewerLease.expiresAt}
            />
        );
    }

    if (loading) {
        return (
            <Box className="flex items-center justify-center" sx={{ minHeight: 360 }}>
                <CircularProgress />
            </Box>
        );
    }

    return (
        <Box sx={{ p: { xs: 2, md: 3 }, width: "100%", maxWidth: 1920, mx: "auto" }}>
            <Paper
                elevation={0}
                sx={{
                    mb: 3,
                    p: { xs: 2, md: 3 },
                    color: "common.white",
                    background: "linear-gradient(135deg, #172554 0%, #1d4ed8 58%, #0891b2 100%)",
                    borderRadius: 3,
                }}
            >
                <Box className="flex items-start justify-between" sx={{ gap: 2, flexWrap: "wrap", mb: 3 }}>
                    <Box>
                        <Typography variant="h5" component="h1" sx={{ fontWeight: 700 }}>
                            Centro de control del enjambre
                        </Typography>
                        <Typography variant="body2" sx={{ mt: 0.75, color: "rgba(255,255,255,.82)" }}>
                            Controla una sesión ROS/Gazebo aislada y observa solamente el video asignado a tu
                            cuenta.
                        </Typography>
                    </Box>
                    <Chip
                        data-testid="operational-status"
                        label={operationalLabel}
                        color={operationalColor}
                        sx={{ bgcolor: "rgba(255,255,255,.92)", fontWeight: 700 }}
                    />
                </Box>

                <Grid container spacing={1.5} aria-label="Recorrido de control">
                    {workflowSteps.map((step) => {
                        const current = workflowStage === step.number;
                        let stepStatus = "Pendiente";
                        if (current) stepStatus = "Siguiente paso";
                        if (step.completed) stepStatus = "Listo";
                        return (
                            <Grid item xs={12} sm={4} key={step.number}>
                                <Box
                                    data-testid={`workflow-step-${step.number}`}
                                    aria-current={current ? "step" : undefined}
                                    sx={{
                                        p: 1.5,
                                        minHeight: 58,
                                        display: "flex",
                                        alignItems: "center",
                                        gap: 1.25,
                                        border: 1,
                                        borderColor: current
                                            ? "rgba(255,255,255,.9)"
                                            : "rgba(255,255,255,.26)",
                                        bgcolor: step.completed ? "rgba(22,163,74,.35)" : "rgba(15,23,42,.22)",
                                        borderRadius: 2,
                                    }}
                                >
                                    <Box
                                        sx={{
                                            width: 30,
                                            height: 30,
                                            display: "grid",
                                            placeItems: "center",
                                            flex: "0 0 auto",
                                            bgcolor: step.completed ? "success.main" : "rgba(255,255,255,.16)",
                                            borderRadius: "50%",
                                            fontWeight: 700,
                                        }}
                                    >
                                        {step.number}
                                    </Box>
                                    <Box>
                                        <Typography variant="subtitle2">{step.label}</Typography>
                                        <Typography variant="caption" sx={{ color: "rgba(255,255,255,.72)" }}>
                                            {stepStatus}
                                        </Typography>
                                    </Box>
                                </Box>
                            </Grid>
                        );
                    })}
                </Grid>
            </Paper>

            {error && (
                <Alert data-testid="workspace-error" severity="error" aria-live="assertive" sx={{ mb: 2 }}>
                    {error}
                </Alert>
            )}

            <Grid container spacing={3} alignItems="stretch">
                <Grid item xs={12} lg={4} xl={3}>
                    <Paper data-testid="session-panel" elevation={2} sx={{ p: 3, height: "100%" }}>
                        <Box className="flex items-start justify-between" sx={{ mb: 2, gap: 2 }}>
                            <Box>
                                <Typography variant="h6">Sesión de simulación</Typography>
                                <Typography variant="body2" color="text.secondary">
                                    1 · Crea y administra tu sesión aislada
                                </Typography>
                            </Box>
                            {displayedSession && (
                                <Box sx={{ textAlign: "right" }}>
                                    <Chip
                                        data-testid="session-state"
                                        label={
                                            SESSION_STATE_LABELS[displayedSession.state] ||
                                            displayedSession.state
                                        }
                                        color={stateColor(displayedSession.state)}
                                        size="small"
                                    />
                                    <Typography
                                        variant="caption"
                                        color="text.secondary"
                                        sx={{ display: "block", mt: 0.5 }}
                                    >
                                        Estado confirmado por el worker GPU
                                    </Typography>
                                </Box>
                            )}
                        </Box>
                        <Divider sx={{ mb: 2 }} />

                        {!activeSession ? (
                            <>
                                {cleanupSession ? (
                                    <>
                                        <Alert severity="error" sx={{ mb: 2 }}>
                                            {cleanupSession.failureReason ||
                                                "La simulación anterior terminó antes de liberar sus recursos GPU."}
                                        </Alert>
                                        <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                                            La limpieza debe finalizar antes de que esta cuenta pueda crear
                                            otra simulación.
                                        </Typography>
                                        <Button
                                            fullWidth
                                            variant="contained"
                                            color="warning"
                                            onClick={retryCleanup}
                                            disabled={submitting}
                                        >
                                            {submitting ? "Limpiando…" : "Reintentar limpieza"}
                                        </Button>
                                    </>
                                ) : (
                                    <>
                                        <Typography variant="body2" color="text.secondary">
                                            La cuenta recibe un entorno ROS/Gazebo independiente en el worker
                                            GPU.
                                        </Typography>
                                        <Box sx={{ px: 1, mt: 4, mb: 2 }}>
                                            <Typography gutterBottom>Robots: {robotCount}</Typography>
                                            <Typography variant="caption" color="text.secondary">
                                                Cantidad inicial del enjambre
                                            </Typography>
                                            <Slider
                                                aria-label="Cantidad inicial de robots"
                                                value={robotCount}
                                                min={1}
                                                max={maxRobotCount}
                                                step={1}
                                                marks={maxRobotCount <= 20}
                                                onChange={(_, value) => setRobotCount(value)}
                                                valueLabelDisplay="auto"
                                            />
                                        </Box>
                                        <Button
                                            fullWidth
                                            variant="contained"
                                            onClick={createSession}
                                            disabled={submitting}
                                        >
                                            {submitting ? "Creando…" : "Crear simulación"}
                                        </Button>
                                        <Typography
                                            variant="caption"
                                            color="text.secondary"
                                            sx={{ display: "block", mt: 1 }}
                                        >
                                            Crear simulación
                                        </Typography>
                                    </>
                                )}
                            </>
                        ) : (
                            <>
                                <Box sx={{ display: "grid", gap: 0.75 }}>
                                    <Typography variant="body2">
                                        Arena: {activeSession.arenaVersion}
                                    </Typography>
                                    <Typography variant="body2">
                                        Robots activos: {availableRobots.length}
                                    </Typography>
                                    {activeSession.queuePosition && (
                                        <Typography variant="body2">
                                            Posición en cola: {activeSession.queuePosition}
                                        </Typography>
                                    )}
                                    {activeSession.computeWorkerName && (
                                        <Typography variant="body2">
                                            Worker: {activeSession.computeWorkerName}
                                        </Typography>
                                    )}
                                </Box>
                                {activeSession.failureReason && (
                                    <Alert severity="error" sx={{ mt: 2 }}>
                                        {activeSession.failureReason}
                                    </Alert>
                                )}

                                <Box sx={{ px: 1, mt: 3 }}>
                                    <Typography gutterBottom>Flota solicitada: {fleetCount}</Typography>
                                    <Slider
                                        aria-label="Tamaño solicitado de la flota"
                                        value={fleetCount}
                                        min={1}
                                        max={maxRobotCount}
                                        step={1}
                                        marks={maxRobotCount <= 20}
                                        onChange={(_, value) => setFleetCount(value)}
                                        valueLabelDisplay="auto"
                                        disabled={!canResizeFleet || submitting}
                                    />
                                </Box>
                                <Button
                                    fullWidth
                                    variant="outlined"
                                    onClick={updateFleet}
                                    disabled={
                                        !canResizeFleet ||
                                        submitting ||
                                        fleetCount === activeSession.desiredRobotCount ||
                                        activeSession.isEmergencyStopped
                                    }
                                >
                                    Aplicar tamaño de flota
                                </Button>

                                <Button
                                    fullWidth
                                    color={activeSession.isEmergencyStopped ? "warning" : "error"}
                                    variant={activeSession.isEmergencyStopped ? "outlined" : "contained"}
                                    sx={{ mt: 2 }}
                                    onClick={changeEmergencyStop}
                                    disabled={submitting || !activeSession.computeWorkerId}
                                >
                                    {activeSession.isEmergencyStopped
                                        ? "Restablecer parada de emergencia"
                                        : "Parada de emergencia"}
                                </Button>

                                <Button
                                    fullWidth
                                    color="error"
                                    variant="text"
                                    sx={{ mt: 1 }}
                                    onClick={stopSession}
                                    disabled={submitting || activeSession.state === "Stopping"}
                                >
                                    {activeSession.state === "Stopping" ? "Deteniendo…" : "Detener sesión"}
                                </Button>
                                <Typography
                                    variant="caption"
                                    color="text.secondary"
                                    sx={{ display: "block", textAlign: "center" }}
                                >
                                    Detener sesión y liberar recursos
                                </Typography>
                            </>
                        )}
                    </Paper>
                </Grid>

                <Grid item xs={12} lg={8} xl={9}>
                    <Paper
                        data-testid="viewer-panel"
                        elevation={2}
                        sx={{
                            p: { xs: 2, md: 3 },
                            height: "100%",
                            display: "flex",
                            flexDirection: "column",
                        }}
                    >
                        <Box
                            className="flex items-start justify-between"
                            sx={{ mb: 2, gap: 2, flexWrap: "wrap" }}
                        >
                            <Box>
                                <Typography variant="h6">Visor privado de Gazebo</Typography>
                                <Typography variant="body2" color="text.secondary">
                                    2 · Abre o renueva el video exclusivo de esta sesión
                                </Typography>
                            </Box>
                            <Box className="flex items-start" sx={{ gap: 1, flexWrap: "wrap" }}>
                                <FormControl size="small" sx={{ minWidth: 170 }}>
                                    <InputLabel id="viewer-source-label">Vista</InputLabel>
                                    <Select
                                        labelId="viewer-source-label"
                                        value={viewerSource}
                                        label="Vista"
                                        onChange={(event) => {
                                            setViewerSource(event.target.value);
                                            setViewerLease(null);
                                        }}
                                        disabled={!canControl}
                                    >
                                        <MenuItem value="Scene">Vista general de Gazebo</MenuItem>
                                    </Select>
                                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5 }}>
                                        Vista general de la arena
                                    </Typography>
                                </FormControl>
                                {viewerSource === "RobotCamera" && (
                                    <FormControl size="small" sx={{ minWidth: 135 }}>
                                        <InputLabel id="viewer-robot-label">Robot</InputLabel>
                                        <Select
                                            labelId="viewer-robot-label"
                                            value={viewerRobotId}
                                            label="Robot"
                                            onChange={(event) => {
                                                setViewerRobotId(event.target.value);
                                                setViewerLease(null);
                                            }}
                                            disabled={!canControl || availableRobots.length === 0}
                                        >
                                            {availableRobots.map((robot) => (
                                                <MenuItem key={robot.id} value={robot.runtimeId}>
                                                    {robot.runtimeId}
                                                </MenuItem>
                                            ))}
                                        </Select>
                                    </FormControl>
                                )}
                                <Button
                                    variant="outlined"
                                    onClick={openViewer}
                                    disabled={
                                        submitting ||
                                        viewerCommandPending ||
                                        !canControl ||
                                        (viewerSource === "RobotCamera" && !viewerRobotId)
                                    }
                                >
                                    Abrir visor
                                </Button>
                            </Box>
                        </Box>

                        <Box
                            aria-label="Área del visor privado"
                            sx={{
                                width: "100%",
                                height: "auto",
                                aspectRatio: "16 / 9",
                                minHeight: { xs: 260, sm: 340, lg: 480 },
                                flex: "0 0 auto",
                                bgcolor: "#111827",
                                color: "grey.300",
                                borderRadius: 2,
                                overflow: "hidden",
                                display: "flex",
                                alignItems: "center",
                                justifyContent: "center",
                                textAlign: "center",
                                boxShadow: "inset 0 0 0 1px rgba(255,255,255,.08)",
                            }}
                        >
                            {viewerContent}
                        </Box>
                        <Typography
                            variant="caption"
                            color="text.secondary"
                            sx={{ display: "block", mt: 1.25 }}
                        >
                            Este video utiliza un display privado del worker. No tiene que aparecer en la
                            sesión VNC compartida del host.
                        </Typography>
                    </Paper>
                </Grid>

                {activeSession && (
                    <Grid item xs={12}>
                        <Paper elevation={2} sx={{ p: { xs: 2, md: 3 } }}>
                            <SwarmTaskPanel
                                session={activeSession}
                                tasks={tasks}
                                busy={submitting}
                                onStart={startTask}
                                onTaskAction={changeTask}
                            />
                        </Paper>
                    </Grid>
                )}
            </Grid>
        </Box>
    );
}

export default SimulationWorkspace;
