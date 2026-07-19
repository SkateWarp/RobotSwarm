import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    Box,
    Button,
    Chip,
    CircularProgress,
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
import SwarmTaskPanel from "./SwarmTaskPanel";
import WhepViewer from "./WhepViewer";

const TERMINAL_STATES = new Set(["Stopped", "Failed", "Expired"]);
const TERMINAL_TASK_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const UNAVAILABLE_ROBOT_STATES = new Set(["Removed", "Failed"]);
const CLEANUP_STATES = new Set(["Failed", "Expired"]);

const stateColor = (state) => {
    if (state === "Ready" || state === "Active") return "success";
    if (state === "Failed" || state === "Expired") return "error";
    if (state === "Stopping" || state === "Paused") return "warning";
    return "info";
};

const requestMessage = (requestError, fallback) => {
    const response = requestError.response?.data;
    const validationMessage = response?.errors ? Object.values(response.errors).flat()[0] : null;
    return response?.message || validationMessage || fallback;
};

const viewerPlaceholder = (lease, session, cleanupPending) => {
    if (lease) {
        return {
            title: "The viewer route is not commissioned yet",
            body: "The lease is valid, but the public WebRTC route or GPU publisher still needs to be configured.",
        };
    }
    if (session) {
        return {
            title: "Open your session-owned viewer",
            body: "Each lease is short-lived and restricted to the current user and simulation session.",
        };
    }
    if (cleanupPending) {
        return {
            title: "Clean up the previous session to continue",
            body: "The private viewer will be available after the GPU worker confirms that cleanup finished.",
        };
    }
    return {
        title: "Create a session to reserve a private viewer",
        body: "Each lease is short-lived and restricted to the current user and simulation session.",
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
    const viewerPrompt = viewerPlaceholder(viewerLease, activeSession, Boolean(cleanupSession));

    const refreshSessions = useCallback(async () => {
        try {
            const result = await SimulationSessionService.list();
            setSessions(result);
            setError("");
        } catch (requestError) {
            setError(requestMessage(requestError, "The simulation service could not be reached."));
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
            setError(requestMessage(requestError, "The session details could not be refreshed."));
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
            if (event?.error) {
                setError(event.error);
            }
            refresh();
        });
        connection.onreconnected(() => {
            joinSession().catch(() => {});
        });

        connection
            .start()
            .then(joinSession)
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
            "The simulation session could not be created."
        );

    const stopSession = () => {
        if (!activeSession) return;
        setViewerLease(null);
        runRequest(
            () => SimulationSessionService.stop(activeSession.id),
            "The simulation session could not be stopped."
        );
    };

    const retryCleanup = () => {
        if (!cleanupSession) return;
        setViewerLease(null);
        runRequest(
            () => SimulationSessionService.stop(cleanupSession.id),
            "The GPU session cleanup could not be retried."
        );
    };

    const updateFleet = () => {
        if (!activeSession) return;
        runRequest(
            () => SimulationSessionService.updateFleet(activeSession.id, fleetCount),
            "The fleet size could not be updated."
        );
    };

    const startTask = (type, parameters) => {
        if (!activeSession) return;
        runRequest(
            () => SimulationSessionService.startTask(activeSession.id, type, parameters),
            "The swarm task could not be started."
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
        runRequest(() => actions[action](activeSession.id, taskId), "The task command could not be sent.");
    };

    const changeEmergencyStop = () => {
        if (!activeSession) return;
        const request = activeSession.isEmergencyStopped
            ? SimulationSessionService.resetEmergencyStop
            : SimulationSessionService.emergencyStop;
        runRequest(() => request(activeSession.id), "The emergency-stop command could not be sent.");
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
            setViewerLease(lease);
        } catch (requestError) {
            setError(requestMessage(requestError, "The private viewer could not be opened."));
        } finally {
            setSubmitting(false);
        }
    };

    if (loading) {
        return (
            <Box className="flex items-center justify-center" sx={{ minHeight: 360 }}>
                <CircularProgress />
            </Box>
        );
    }

    return (
        <Box sx={{ p: { xs: 2, md: 3 } }}>
            {error && (
                <Alert severity="error" sx={{ mb: 2 }}>
                    {error}
                </Alert>
            )}

            <Grid container spacing={3}>
                <Grid item xs={12} lg={7}>
                    <Paper elevation={2} sx={{ p: 2, minHeight: 520 }}>
                        <Box
                            className="flex items-center justify-between"
                            sx={{ mb: 2, gap: 2, flexWrap: "wrap" }}
                        >
                            <Typography variant="h6">Private simulation viewer</Typography>
                            <Box className="flex items-center" sx={{ gap: 1, flexWrap: "wrap" }}>
                                <FormControl size="small" sx={{ minWidth: 170 }}>
                                    <InputLabel id="viewer-source-label">View</InputLabel>
                                    <Select
                                        labelId="viewer-source-label"
                                        value={viewerSource}
                                        label="View"
                                        onChange={(event) => {
                                            setViewerSource(event.target.value);
                                            setViewerLease(null);
                                        }}
                                        disabled={!canControl}
                                    >
                                        <MenuItem value="Scene">Gazebo overview</MenuItem>
                                    </Select>
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
                                        !canControl ||
                                        (viewerSource === "RobotCamera" && !viewerRobotId)
                                    }
                                >
                                    Open viewer
                                </Button>
                            </Box>
                        </Box>

                        <Box
                            sx={{
                                height: 430,
                                bgcolor: "#111827",
                                color: "grey.300",
                                borderRadius: 2,
                                overflow: "hidden",
                                display: "flex",
                                alignItems: "center",
                                justifyContent: "center",
                                textAlign: "center",
                            }}
                        >
                            {viewerLease?.signalingUrl && viewerLease.token ? (
                                <WhepViewer
                                    url={viewerLease.signalingUrl}
                                    token={viewerLease.token}
                                    expiresAt={viewerLease.expiresAt}
                                />
                            ) : (
                                <Box sx={{ px: 4 }}>
                                    <Typography variant="h6" sx={{ mb: 1 }}>
                                        {viewerPrompt.title}
                                    </Typography>
                                    <Typography variant="body2">{viewerPrompt.body}</Typography>
                                </Box>
                            )}
                        </Box>
                    </Paper>
                </Grid>

                <Grid item xs={12} lg={5}>
                    <Paper elevation={2} sx={{ p: 3, mb: 3 }}>
                        <Box className="flex items-center justify-between" sx={{ mb: 2 }}>
                            <Typography variant="h6">Simulation session</Typography>
                            {displayedSession && (
                                <Chip
                                    label={displayedSession.state}
                                    color={stateColor(displayedSession.state)}
                                    size="small"
                                />
                            )}
                        </Box>

                        {!activeSession ? (
                            <>
                                {cleanupSession ? (
                                    <>
                                        <Alert severity="error" sx={{ mb: 2 }}>
                                            {cleanupSession.failureReason ||
                                                "The previous simulation ended before its GPU resources were released."}
                                        </Alert>
                                        <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
                                            Cleanup must finish before this account can start another
                                            simulation.
                                        </Typography>
                                        <Button
                                            fullWidth
                                            variant="contained"
                                            color="warning"
                                            onClick={retryCleanup}
                                            disabled={submitting}
                                        >
                                            {submitting ? "Cleaning up…" : "Retry cleanup"}
                                        </Button>
                                    </>
                                ) : (
                                    <>
                                        <Typography variant="body2" color="text.secondary">
                                            Each account receives an isolated ROS/Gazebo session on the GPU
                                            worker.
                                        </Typography>
                                        <Box sx={{ px: 1, mt: 4, mb: 2 }}>
                                            <Typography gutterBottom>Robots: {robotCount}</Typography>
                                            <Slider
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
                                            {submitting ? "Creating…" : "Create simulation"}
                                        </Button>
                                    </>
                                )}
                            </>
                        ) : (
                            <>
                                <Typography variant="body2">Arena: {activeSession.arenaVersion}</Typography>
                                <Typography variant="body2">
                                    Active robots: {availableRobots.length}
                                </Typography>
                                {activeSession.queuePosition && (
                                    <Typography variant="body2">
                                        Queue position: {activeSession.queuePosition}
                                    </Typography>
                                )}
                                {activeSession.computeWorkerName && (
                                    <Typography variant="body2">
                                        Worker: {activeSession.computeWorkerName}
                                    </Typography>
                                )}
                                {activeSession.failureReason && (
                                    <Alert severity="error" sx={{ mt: 2 }}>
                                        {activeSession.failureReason}
                                    </Alert>
                                )}

                                <Box sx={{ px: 1, mt: 3 }}>
                                    <Typography gutterBottom>Desired fleet: {fleetCount}</Typography>
                                    <Slider
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
                                    Apply fleet size
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
                                        ? "Reset emergency stop"
                                        : "Emergency stop"}
                                </Button>

                                <Button
                                    fullWidth
                                    color="error"
                                    variant="text"
                                    sx={{ mt: 1 }}
                                    onClick={stopSession}
                                    disabled={submitting || activeSession.state === "Stopping"}
                                >
                                    {activeSession.state === "Stopping" ? "Stopping…" : "Stop session"}
                                </Button>
                            </>
                        )}
                    </Paper>

                    {activeSession && (
                        <Paper elevation={2} sx={{ p: 3 }}>
                            <SwarmTaskPanel
                                session={activeSession}
                                tasks={tasks}
                                busy={submitting}
                                onStart={startTask}
                                onTaskAction={changeTask}
                            />
                        </Paper>
                    )}
                </Grid>
            </Grid>
        </Box>
    );
}

export default SimulationWorkspace;
