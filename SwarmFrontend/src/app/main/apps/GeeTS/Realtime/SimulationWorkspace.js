import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    Box,
    Button,
    Chip,
    CircularProgress,
    Divider,
    Dialog,
    DialogActions,
    DialogContent,
    DialogContentText,
    DialogTitle,
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
import PageHeading from "../../../../shared-components/PageHeading";
import HlsViewer from "./HlsViewer";
import SwarmTaskPanel from "./SwarmTaskPanel";
import WhepViewer from "./WhepViewer";
import { parseViewerTimestamp } from "./viewerTimestamp";

const TERMINAL_STATES = new Set(["Stopped", "Failed", "Expired"]);
const TERMINAL_TASK_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const TERMINAL_VIEWER_COMMAND_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const UNAVAILABLE_ROBOT_STATES = new Set(["Removed", "Failed"]);
const CLEANUP_STATES = new Set(["Failed", "Expired"]);

const SESSION_ROBOT_STATE_LABELS = {
    Provisioning: "Preparando",
    Ready: "Listo",
    Active: "Activo",
    Offline: "Sin conexión",
    Removed: "Retirado",
    Failed: "Fallido",
};

export const sessionRobotStateColor = (state) => {
    if (state === "Ready" || state === "Active") return "success";
    if (state === "Offline" || state === "Removed") return "warning";
    if (state === "Failed") return "error";
    return "info";
};

export const sessionRobotSummary = (robots) => {
    const list = Array.isArray(robots) ? robots : [];
    return {
        total: list.length,
        operational: list.filter((robot) => ["Ready", "Active"].includes(robot.state)).length,
        provisioning: list.filter((robot) => robot.state === "Provisioning").length,
        unavailable: list.filter((robot) => ["Offline", "Removed", "Failed"].includes(robot.state)).length,
    };
};

const formatRobotUpdate = (value) => {
    const date = new Date(parseViewerTimestamp(value));
    if (Number.isNaN(date.getTime())) return "sin actualización";
    return new Intl.DateTimeFormat("es-BO", {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
    }).format(date);
};

const commandTimestamp = (command) => {
    const value = parseViewerTimestamp(command?.updatedAt || command?.createdAt);
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

export const viewerCloseOutcomeNotice = (outcome) => {
    if (outcome?.state === "Failed") {
        const detail = outcome.command?.error ? `: ${outcome.command.error}` : ".";
        return {
            severity: "error",
            message: `El worker no pudo cerrar el proceso del visor${detail} El acceso privado ya fue revocado, pero la liberación del publicador sigue sin confirmarse. Puedes reintentar sin detener la sesión.`,
        };
    }
    if (outcome?.state === "Cancelled") {
        return {
            severity: "warning",
            message:
                "El cierre del visor fue cancelado por el worker. El acceso privado ya fue revocado; reintenta para confirmar la liberación del publicador.",
        };
    }
    return {
        severity: "warning",
        message:
            "El acceso privado ya fue revocado, pero el worker aún no confirmó que liberó el publicador. Puedes volver a comprobar el cierre sin detener la sesión ROS.",
    };
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
    const [closingViewer, setClosingViewer] = useState(false);
    const [viewerCloseNotice, setViewerCloseNotice] = useState(null);
    const [connectionVersion, setConnectionVersion] = useState(0);
    const [realtimeStatus, setRealtimeStatus] = useState("Idle");
    const [realtimeRetryVersion, setRealtimeRetryVersion] = useState(0);
    const [stopConfirmationOpen, setStopConfirmationOpen] = useState(false);
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
    const robotSummary = useMemo(() => sessionRobotSummary(robots), [robots]);
    const canControl = Boolean(activeSession && ["Ready", "Active", "Paused"].includes(activeSession.state));
    const canResizeFleet = Boolean(
        activeSession?.state === "Ready" && !tasks.some((task) => !TERMINAL_TASK_STATES.has(task.state))
    );
    const viewerCommandState = viewerLease?.command?.state;
    const viewerCommandPending = Boolean(
        viewerLease?.command && !TERMINAL_VIEWER_COMMAND_STATES.has(viewerCommandState)
    );
    const viewerReady = viewerCommandState === "Completed" && !viewerLease?.revokedAt;
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
        setViewerCloseNotice(null);
    }, [viewerLease?.token]);

    useEffect(() => {
        if (!activeSessionId) {
            setRealtimeStatus("Idle");
            return undefined;
        }

        let disposed = false;
        let retryTimer;
        let retryAttempt = 0;
        const connection = SimulationSessionService.createRealtimeConnection();
        connectionRef.current = connection;

        const refresh = () => {
            refreshSessions();
            refreshSessionDetails(activeSessionId);
        };
        const joinSession = async () => {
            if (disposed) return;
            await connection.invoke("JoinSession", activeSessionId);
            joinedSessionRef.current = activeSessionId;
        };
        const scheduleStart = (delay = 3000) => {
            if (disposed) return;
            window.clearTimeout(retryTimer);
            setRealtimeStatus("Retrying");
            retryTimer = window.setTimeout(() => {
                startConnection();
            }, delay);
        };
        const startConnection = async () => {
            if (disposed) return;
            setRealtimeStatus(retryAttempt === 0 ? "Connecting" : "Retrying");
            try {
                await connection.start();
                await joinSession();
                if (!disposed) {
                    retryAttempt = 0;
                    setRealtimeStatus("Connected");
                    setConnectionVersion((current) => current + 1);
                }
            } catch (_requestError) {
                if (disposed) return;
                retryAttempt += 1;
                if (connection.state !== "Disconnected") {
                    await connection.stop().catch(() => {});
                }
                scheduleStart(Math.min(10000, 1000 * 2 ** Math.min(retryAttempt, 3)));
            }
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
                .then(() => {
                    setRealtimeStatus("Connected");
                    setConnectionVersion((current) => current + 1);
                })
                .catch(() => {
                    connection.stop().catch(() => {});
                    scheduleStart();
                });
        });
        connection.onreconnecting(() => {
            setViewerControlReady(false);
            setRealtimeStatus("Retrying");
        });
        connection.onclose(() => {
            if (!disposed) {
                setViewerControlReady(false);
                scheduleStart(3000);
            }
        });

        startConnection();

        return () => {
            disposed = true;
            window.clearTimeout(retryTimer);
            const joinedSessionId = joinedSessionRef.current;
            if (joinedSessionId && connection.state === "Connected") {
                connection.invoke("LeaveSession", joinedSessionId).catch(() => {});
            }
            connection.stop().catch(() => {});
            connectionRef.current = null;
            joinedSessionRef.current = null;
        };
    }, [activeSessionId, realtimeRetryVersion, refreshSessionDetails, refreshSessions]);

    useEffect(() => {
        if (closingViewer || !activeSessionId || !viewerLease?.leaseId || !viewerReady) {
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
                    const expiresAt = parseViewerTimestamp(authorization?.authorizedUntil);
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
    }, [activeSessionId, closingViewer, connectionVersion, viewerLease?.leaseId, viewerReady]);

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
        setStopConfirmationOpen(true);
    };

    const confirmStopSession = () => {
        if (!activeSession) return;
        setStopConfirmationOpen(false);
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

    const closeViewer = async () => {
        if (!activeSessionId || !viewerLease?.leaseId || closingViewer) {
            return;
        }

        const { leaseId } = viewerLease;
        const connection = connectionRef.current;
        const releaseInput =
            viewerControlReady && connection?.state === "Connected"
                ? sendViewerControlInput(connection, activeSessionId, leaseId, { type: "releaseAll" }).catch(
                      () => {}
                  )
                : Promise.resolve();
        setClosingViewer(true);
        setViewerControlReady(false);
        setViewerCloseNotice(null);
        setError("");
        try {
            await releaseInput;
            const response = await SimulationSessionService.closeViewerLease(activeSessionId, leaseId);
            const initialState = response?.command?.state;
            if (initialState === "Completed" || (!response?.accepted && !response?.command)) {
                setHlsUnavailable(false);
                setViewerLease((current) => (current?.leaseId === leaseId ? null : current));
                return;
            }

            setViewerLease((current) =>
                current?.leaseId === leaseId
                    ? {
                          ...current,
                          revokedAt: response.revokedAt || new Date().toISOString(),
                          closeCommand: response.command,
                      }
                    : current
            );

            const outcome = TERMINAL_VIEWER_COMMAND_STATES.has(initialState)
                ? { state: initialState, command: response.command }
                : await SimulationSessionService.waitForViewerClose(activeSessionId, leaseId);
            if (outcome.state === "Completed") {
                setHlsUnavailable(false);
                setViewerLease((current) => (current?.leaseId === leaseId ? null : current));
                return;
            }

            setViewerLease((current) =>
                current?.leaseId === leaseId
                    ? { ...current, closeCommand: outcome.command || current.closeCommand }
                    : current
            );
            setViewerCloseNotice(viewerCloseOutcomeNotice(outcome));
        } catch (requestError) {
            setError(requestMessage(requestError, "No fue posible cerrar el visor privado."));
        } finally {
            setClosingViewer(false);
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
    let closeViewerLabel = "Cerrar visor";
    if (closingViewer) {
        closeViewerLabel = "Confirmando cierre…";
    } else if (viewerCloseNotice) {
        closeViewerLabel = "Reintentar cierre";
    }

    let viewerContent = (
        <Box sx={{ px: 4 }}>
            <Typography variant="h6" sx={{ mb: 1 }}>
                {viewerPrompt.title}
            </Typography>
            <Typography variant="body2">{viewerPrompt.body}</Typography>
        </Box>
    );
    if (closingViewer || viewerLease?.revokedAt) {
        viewerContent = (
            <Box data-testid="viewer-closing" sx={{ width: "100%", px: 3 }} aria-live="polite">
                {closingViewer ? (
                    <>
                        <CircularProgress size={34} color="inherit" sx={{ mb: 2 }} />
                        <Typography variant="h6" sx={{ mb: 1 }}>
                            Confirmando cierre del visor…
                        </Typography>
                        <Typography variant="body2">
                            El acceso y la interacción ya están deshabilitados. Esperando la confirmación del
                            worker GPU sin detener ROS ni Gazebo.
                        </Typography>
                    </>
                ) : (
                    <Alert
                        data-testid="viewer-close-notice"
                        severity={viewerCloseNotice?.severity || "warning"}
                    >
                        {viewerCloseNotice?.message ||
                            "El cierre fue solicitado y continúa pendiente de confirmación del worker."}
                    </Alert>
                )}
            </Box>
        );
    } else if (viewerCommandPending) {
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
            <Box sx={{ p: { xs: 2, md: 3 }, width: "100%", maxWidth: 1920, mx: "auto" }}>
                <PageHeading
                    title="Control de simulación"
                    description="Administre una sesión ROS/Gazebo aislada y su visor privado."
                    status={<Chip label="Consultando" color="info" size="small" variant="outlined" />}
                />
                <Box
                    role="status"
                    aria-live="polite"
                    className="flex items-center justify-center"
                    sx={{ minHeight: 280, gap: 1.5 }}
                >
                    <CircularProgress size={28} />
                    <Typography color="text.secondary">Consultando sesiones disponibles…</Typography>
                </Box>
            </Box>
        );
    }

    return (
        <Box sx={{ p: { xs: 2, md: 3 }, width: "100%", maxWidth: 1920, mx: "auto" }}>
            <PageHeading
                title="Control de simulación"
                description="Administre una sesión ROS/Gazebo aislada y su visor privado."
                status={
                    <Chip
                        data-testid="operational-status"
                        label={operationalLabel}
                        color={operationalColor}
                        size="small"
                        variant="outlined"
                    />
                }
            >
                <Grid container columnSpacing={3} rowSpacing={1.5} aria-label="Recorrido de control">
                    {workflowSteps.map((step) => {
                        const current = workflowStage === step.number;
                        let stepStatus = "Pendiente";
                        if (current) stepStatus = "Siguiente paso";
                        if (step.completed) stepStatus = "Listo";
                        let stepBorderColor = "divider";
                        if (current) stepBorderColor = "primary.main";
                        else if (step.completed) stepBorderColor = "success.main";
                        return (
                            <Grid item xs={12} sm={4} key={step.number}>
                                <Box
                                    data-testid={`workflow-step-${step.number}`}
                                    aria-current={current ? "step" : undefined}
                                    sx={{
                                        py: 0.5,
                                        pl: 1.5,
                                        minHeight: 42,
                                        display: "flex",
                                        alignItems: "center",
                                        gap: 1.25,
                                        borderLeft: 3,
                                        borderColor: stepBorderColor,
                                    }}
                                >
                                    <Typography
                                        variant="caption"
                                        sx={{
                                            width: 20,
                                            flex: "0 0 auto",
                                            fontWeight: 700,
                                            color: step.completed ? "success.main" : "text.secondary",
                                        }}
                                    >
                                        {String(step.number).padStart(2, "0")}
                                    </Typography>
                                    <Box>
                                        <Typography variant="subtitle2">{step.label}</Typography>
                                        <Typography variant="caption" color="text.secondary">
                                            {stepStatus}
                                        </Typography>
                                    </Box>
                                </Box>
                            </Grid>
                        );
                    })}
                </Grid>
            </PageHeading>

            {error && (
                <Alert data-testid="workspace-error" severity="error" aria-live="assertive" sx={{ mb: 2 }}>
                    {error}
                </Alert>
            )}

            {activeSession && realtimeStatus !== "Connected" && (
                <Alert
                    data-testid="realtime-status"
                    severity="warning"
                    aria-live="polite"
                    sx={{ mb: 2 }}
                    action={
                        <Button
                            color="inherit"
                            size="small"
                            onClick={() => setRealtimeRetryVersion((current) => current + 1)}
                        >
                            Reconectar ahora
                        </Button>
                    }
                >
                    Reconectando el canal en tiempo real. El estado continúa actualizándose por sondeo, pero la
                    interacción con el visor permanece deshabilitada hasta recuperar la conexión.
                </Alert>
            )}

            <Grid container spacing={3} alignItems="stretch">
                <Grid item xs={12} lg={4} xl={3}>
                    <Paper
                        data-testid="session-panel"
                        elevation={0}
                        variant="outlined"
                        sx={{ p: 3, height: "100%" }}
                    >
                        <Box className="flex items-start justify-between" sx={{ mb: 2, gap: 2 }}>
                            <Box>
                                <Typography variant="h6">Sesión de simulación</Typography>
                                <Typography variant="body2" color="text.secondary">
                                    Cree y administre un entorno ROS/Gazebo aislado.
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
                                        <Typography variant="body2" data-sensitive="worker-identity">
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
                        elevation={0}
                        variant="outlined"
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
                                    Abra o renueve el video exclusivo de esta sesión.
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
                                        closingViewer ||
                                        viewerCommandPending ||
                                        Boolean(viewerLease?.revokedAt) ||
                                        !canControl ||
                                        (viewerSource === "RobotCamera" && !viewerRobotId)
                                    }
                                >
                                    Abrir visor
                                </Button>
                                {viewerLease && (
                                    <Button
                                        color="error"
                                        variant="outlined"
                                        onClick={closeViewer}
                                        disabled={closingViewer}
                                    >
                                        {closeViewerLabel}
                                    </Button>
                                )}
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
                                borderRadius: 1,
                                overflow: "hidden",
                                display: "flex",
                                alignItems: "center",
                                justifyContent: "center",
                                textAlign: "center",
                                border: "1px solid rgba(255,255,255,.12)",
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
                        <Paper
                            data-testid="session-robot-monitor"
                            elevation={0}
                            variant="outlined"
                            sx={{ p: { xs: 2, md: 3 } }}
                        >
                            <Box
                                className="flex items-start justify-between"
                                sx={{ gap: 2, flexWrap: "wrap", mb: 2 }}
                            >
                                <Box>
                                    <Typography variant="h6">Robots de esta sesión</Typography>
                                    <Typography variant="body2" color="text.secondary">
                                        Instancias reales informadas por el worker para este entorno
                                        ROS/Gazebo.
                                    </Typography>
                                </Box>
                                <Box className="flex items-center" sx={{ gap: 1, flexWrap: "wrap" }}>
                                    <Chip
                                        size="small"
                                        color={realtimeStatus === "Connected" ? "success" : "warning"}
                                        label={
                                            realtimeStatus === "Connected"
                                                ? "Tiempo real conectado"
                                                : "Actualización por sondeo"
                                        }
                                    />
                                    <Chip
                                        size="small"
                                        variant="outlined"
                                        label={`${robotSummary.operational}/${robotSummary.total} operativos`}
                                    />
                                </Box>
                            </Box>

                            {robots.length === 0 && (
                                <Alert severity="info">
                                    El worker todavía no ha publicado el roster de robots de esta sesión.
                                </Alert>
                            )}
                            {robotSummary.unavailable > 0 && (
                                <Alert severity="warning" sx={{ mb: 2 }}>
                                    {robotSummary.unavailable} robot(es) requieren atención. Revise su estado
                                    antes de iniciar otra tarea.
                                </Alert>
                            )}

                            {robots.length > 0 && (
                                <Grid container spacing={1.5}>
                                    {robots.map((robot) => (
                                        <Grid item xs={12} sm={6} md={4} xl={3} key={robot.id}>
                                            <Paper variant="outlined" sx={{ p: 1.5, height: "100%" }}>
                                                <Box
                                                    className="flex items-start justify-between"
                                                    sx={{ gap: 1, mb: 0.75 }}
                                                >
                                                    <Box sx={{ minWidth: 0 }}>
                                                        <Typography variant="subtitle2" noWrap>
                                                            {robot.runtimeId}
                                                        </Typography>
                                                        <Typography
                                                            variant="caption"
                                                            color="text.secondary"
                                                            noWrap
                                                        >
                                                            {robot.namespace}
                                                        </Typography>
                                                    </Box>
                                                    <Chip
                                                        size="small"
                                                        color={sessionRobotStateColor(robot.state)}
                                                        label={
                                                            SESSION_ROBOT_STATE_LABELS[robot.state] ||
                                                            robot.state
                                                        }
                                                    />
                                                </Box>
                                                <Typography variant="caption" color="text.secondary">
                                                    #{robot.ordinal + 1} · rol {robot.role || "sin asignar"} ·
                                                    actualizado {formatRobotUpdate(robot.updatedAt)}
                                                </Typography>
                                            </Paper>
                                        </Grid>
                                    ))}
                                </Grid>
                            )}
                        </Paper>
                    </Grid>
                )}

                {activeSession && (
                    <Grid item xs={12}>
                        <Paper elevation={0} variant="outlined" sx={{ p: { xs: 2, md: 3 } }}>
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

            <Dialog
                open={stopConfirmationOpen}
                onClose={submitting ? undefined : () => setStopConfirmationOpen(false)}
                fullWidth
                maxWidth="xs"
            >
                <DialogTitle>Detener sesión de simulación</DialogTitle>
                <DialogContent>
                    <DialogContentText>
                        Se cancelará cualquier tarea activa y se liberarán el visor, Gazebo, ROS, el contenedor
                        y la red privada de esta sesión. Esta acción no se puede deshacer.
                    </DialogContentText>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setStopConfirmationOpen(false)} disabled={submitting}>
                        Conservar sesión
                    </Button>
                    <Button
                        color="error"
                        variant="contained"
                        onClick={confirmStopSession}
                        disabled={submitting}
                    >
                        {submitting ? "Deteniendo…" : "Detener y liberar"}
                    </Button>
                </DialogActions>
            </Dialog>
        </Box>
    );
}

export default SimulationWorkspace;
