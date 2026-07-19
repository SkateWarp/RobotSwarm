import { useState } from "react";
import PropTypes from "prop-types";
import {
    Alert,
    Box,
    Button,
    Chip,
    FormControl,
    Grid,
    InputLabel,
    LinearProgress,
    MenuItem,
    Select,
    TextField,
    Typography,
} from "@mui/material";
import transportTaskNotice from "./transportTaskStatus";

const TASK_TYPES = [
    { value: "FollowLeader", label: "Follow the leader" },
    { value: "Figure", label: "Figure or letter" },
    { value: "CollaborativeTransport", label: "Collaborative transport" },
];

const TERMINAL_TASK_STATES = new Set(["Completed", "Cancelled", "Failed"]);
const SUPPORTED_LETTERS = "ABCDEFGHIJKLMNOPRSTUVWXYZ".split("");

const clampNumber = (value, minimum, maximum, fallback) => {
    const parsed = Number(value);
    if (!Number.isFinite(parsed)) return fallback;
    return Math.min(maximum, Math.max(minimum, parsed));
};

const taskColor = (state) => {
    if (state === "Running" || state === "Completed") return "success";
    if (state === "Paused" || state === "Cancelling") return "warning";
    if (state === "Failed") return "error";
    return "info";
};

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
    const transportNotice = transportTaskNotice(latestTask);
    const verifiedOutcome = latestTask?.outcomeState;

    const buildParameters = () => {
        if (taskType === "FollowLeader") {
            return {
                leader_mode: leaderMode,
                config: {
                    leader_mode: leaderMode,
                    follow_distance: clampNumber(followDistance, 0.35, 2, 0.7),
                    radius: clampNumber(pathRadius, 0.5, 4, 2),
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
                    spacing: clampNumber(formationSpacing, 0.35, 2, 0.7),
                },
            };
        }

        return {
            target_x: clampNumber(targetX, -4, 4, 3),
            target_y: clampNumber(targetY, -4, 4, 3),
            config: {
                target_x: clampNumber(targetX, -4, 4, 3),
                target_y: clampNumber(targetY, -4, 4, 3),
                transport_planner: "grf",
            },
        };
    };

    return (
        <>
            <Box className="flex items-center justify-between" sx={{ mb: 2 }}>
                <Typography variant="h6">Swarm task</Typography>
                {latestTask && (
                    <Chip label={latestTask.state} color={taskColor(latestTask.state)} size="small" />
                )}
            </Box>

            {!canControl && (
                <Alert severity="info" sx={{ mb: 2 }}>
                    Task controls become available when the GPU worker reports the session ready.
                </Alert>
            )}

            {session.isEmergencyStopped && (
                <Alert severity="error" sx={{ mb: 2 }}>
                    Emergency stop is active. Reset it before starting or resuming a task.
                </Alert>
            )}

            {latestTask && (
                <Box sx={{ mb: 3 }}>
                    <Box className="flex items-center justify-between" sx={{ mb: 1 }}>
                        <Typography variant="body2">
                            {TASK_TYPES.find((item) => item.value === latestTask.type)?.label ||
                                latestTask.type}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                            {Math.round(latestProgress * 100)}%
                        </Typography>
                    </Box>
                    <LinearProgress variant="determinate" value={latestProgress * 100} />
                    {verifiedOutcome && verifiedOutcome !== "Pending" && (
                        <Typography variant="caption" color="text.secondary" sx={{ mt: 1, display: "block" }}>
                            Verified outcome: {verifiedOutcome}
                        </Typography>
                    )}
                    {latestTask.error && (
                        <Alert severity="error" sx={{ mt: 2 }}>
                            {latestTask.error}
                        </Alert>
                    )}
                    {transportNotice && (
                        <Alert severity={transportNotice.severity} sx={{ mt: 2 }}>
                            {transportNotice.message}
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
                                Pause
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
                                Resume
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
                            {activeTask.state === "Cancelling" ? "Cancelling…" : "Cancel task"}
                        </Button>
                    </Grid>
                </Grid>
            ) : (
                <>
                    <FormControl fullWidth size="small" sx={{ mb: 2 }}>
                        <InputLabel id="task-type-label">Task</InputLabel>
                        <Select
                            labelId="task-type-label"
                            value={taskType}
                            label="Task"
                            onChange={(event) => setTaskType(event.target.value)}
                        >
                            {TASK_TYPES.map((item) => (
                                <MenuItem key={item.value} value={item.value}>
                                    {item.label}
                                </MenuItem>
                            ))}
                        </Select>
                    </FormControl>

                    {taskType === "FollowLeader" && (
                        <Grid container spacing={2}>
                            <Grid item xs={12}>
                                <FormControl fullWidth size="small">
                                    <InputLabel id="leader-mode-label">Leader path</InputLabel>
                                    <Select
                                        labelId="leader-mode-label"
                                        value={leaderMode}
                                        label="Leader path"
                                        onChange={(event) => setLeaderMode(event.target.value)}
                                    >
                                        <MenuItem value="circular">Circle</MenuItem>
                                        <MenuItem value="square">Square</MenuItem>
                                        <MenuItem value="figure8">Figure eight</MenuItem>
                                    </Select>
                                </FormControl>
                            </Grid>
                            <Grid item xs={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Follower gap (m)"
                                    value={followDistance}
                                    inputProps={{ min: 0.35, max: 2, step: 0.05 }}
                                    onChange={(event) => setFollowDistance(event.target.value)}
                                />
                            </Grid>
                            <Grid item xs={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Path radius (m)"
                                    value={pathRadius}
                                    inputProps={{ min: 0.5, max: 4, step: 0.25 }}
                                    onChange={(event) => setPathRadius(event.target.value)}
                                />
                            </Grid>
                        </Grid>
                    )}

                    {taskType === "Figure" && (
                        <Grid container spacing={2}>
                            <Grid item xs={12} sm={6}>
                                <FormControl fullWidth size="small">
                                    <InputLabel id="formation-type-label">Figure</InputLabel>
                                    <Select
                                        labelId="formation-type-label"
                                        value={formationType}
                                        label="Figure"
                                        onChange={(event) => setFormationType(event.target.value)}
                                    >
                                        <MenuItem value="circle">Circle</MenuItem>
                                        <MenuItem value="square">Square</MenuItem>
                                        <MenuItem value="triangle">Triangle</MenuItem>
                                        <MenuItem value="diamond">Diamond</MenuItem>
                                        <MenuItem value="line">Line</MenuItem>
                                        <MenuItem value="v_formation">V formation</MenuItem>
                                        <MenuItem value="letter">Letter</MenuItem>
                                    </Select>
                                </FormControl>
                            </Grid>
                            {formationType === "letter" && (
                                <Grid item xs={12} sm={6}>
                                    <FormControl fullWidth size="small">
                                        <InputLabel id="formation-letter-label">Letter</InputLabel>
                                        <Select
                                            labelId="formation-letter-label"
                                            value={formationLetter}
                                            label="Letter"
                                            onChange={(event) => {
                                                setFormationLetter(event.target.value);
                                            }}
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
                                <TextField fullWidth size="small" label="Movement" value="Static" disabled />
                            </Grid>
                            <Grid item xs={12} sm={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Robot spacing (m)"
                                    value={formationSpacing}
                                    inputProps={{ min: 0.35, max: 2, step: 0.05 }}
                                    onChange={(event) => setFormationSpacing(event.target.value)}
                                />
                            </Grid>
                        </Grid>
                    )}

                    {taskType === "CollaborativeTransport" && (
                        <Grid container spacing={2}>
                            <Grid item xs={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Target X"
                                    value={targetX}
                                    inputProps={{ min: -4, max: 4, step: 0.25 }}
                                    onChange={(event) => setTargetX(event.target.value)}
                                />
                            </Grid>
                            <Grid item xs={6}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    type="number"
                                    label="Target Y"
                                    value={targetY}
                                    inputProps={{ min: -4, max: 4, step: 0.25 }}
                                    onChange={(event) => setTargetY(event.target.value)}
                                />
                            </Grid>
                            <Grid item xs={12}>
                                <TextField
                                    fullWidth
                                    size="small"
                                    label="Planner"
                                    value="Coordinated GRF"
                                    disabled
                                />
                            </Grid>
                        </Grid>
                    )}

                    <Button
                        fullWidth
                        variant="contained"
                        sx={{ mt: 2 }}
                        onClick={() => onStart(taskType, buildParameters())}
                        disabled={busy || !canStart}
                    >
                        {busy ? "Sending…" : "Start task"}
                    </Button>
                </>
            )}
        </>
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
