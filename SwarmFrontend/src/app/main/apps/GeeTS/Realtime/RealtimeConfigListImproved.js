import { useState, useEffect, useRef } from "react";
import {
    Box,
    Grid,
    Paper,
    Typography,
    Alert,
    Snackbar
} from "@mui/material";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import VncViewer from "./VncViewer";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
import useLoggedUserId from "../../../../shared-components/hooks/useLoggedUserId";
import CommandPanel from "./CommandPanel";
import CommandStatus from "./CommandStatus";

function RealtimeConfigListImproved() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState("");
    const [connectionStatus, setConnectionStatus] = useState("disconnected");
    const [commandFeedback, setCommandFeedback] = useState({ open: false, message: "", severity: "info" });
    const [runningCommands, setRunningCommands] = useState({});
    const [commandHistory, setCommandHistory] = useState([]);

    const userId = useLoggedUserId();
    const connectionRef = useRef(null);

    useEffect(() => {
        setupSignalRConnection();
        fetchUserRobots();
        loadRunningTaskLogs();

        return () => {
            cleanupConnection();
        };
    }, [userId]);

    const setupSignalRConnection = async () => {
        try {
            setConnectionStatus("connecting");
            connectionRef.current = singletonInstance.createConnectionBuilder();
            await singletonInstance.getConnectionPromise();
            setupEventHandlers();
            setConnectionStatus("connected");
            console.log("SignalR connection established successfully");
        } catch (error) {
            console.error("Error establishing SignalR connection:", error);
            setConnectionStatus("error");
        }
    };

    const setupEventHandlers = () => {
        if (!connectionRef.current) return;

        connectionRef.current.on("ExecuteCommand", (commandData) => {
            console.log("Command executed:", commandData);
            showFeedback(`Command executed: ${commandData.command}`, "success");
        });

        connectionRef.current.onreconnecting(() => setConnectionStatus("reconnecting"));
        connectionRef.current.onreconnected(() => setConnectionStatus("connected"));
        connectionRef.current.onclose(() => setConnectionStatus("disconnected"));
    };

    const fetchUserRobots = async () => {
        try {
            const response = await axios.get(`${URL}/Robots`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            const accessibleRobots = response.data.filter(robot =>
                robot.accountId === userId || robot.isPublic
            );

            setRobots(accessibleRobots);
        } catch (error) {
            console.error("Error fetching robots:", error);
            showFeedback("Error fetching robots", "error");
        }
    };

    const loadRunningTaskLogs = async () => {
        try {
            const response = await axios.get(`${URL}/TaskLog`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            const taskLogs = response.data;
            const runningTaskLogs = taskLogs.filter(task =>
                task.dateFinished === null &&
                task.dateCancelled === null &&
                task.robots?.some(robot => robot.accountId === userId)
            );

            const runningState = {};
            runningTaskLogs.forEach(task => {
                task.robots.forEach(robot => {
                    if (robot.accountId === userId) {
                        runningState[robot.id] = {
                            id: `task_${task.id}`,
                            taskLogId: task.id,
                            command: task.taskTemplate?.name || "Unknown",
                            parameters: task.parameters,
                            startTime: new Date(task.dateCreated),
                            status: "running"
                        };
                    }
                });
            });

            setRunningCommands(runningState);
        } catch (error) {
            console.error("Error loading running TaskLogs:", error);
        }
    };

    const cleanupConnection = () => {
        if (connectionRef.current) {
            singletonInstance.stopConnection();
            connectionRef.current = null;
        }
    };

    const showFeedback = (message, severity = "info") => {
        setCommandFeedback({ open: true, message, severity });
    };

    const handleCloseFeedback = () => {
        setCommandFeedback({ ...commandFeedback, open: false });
    };

    const stopRunningTasksForRobot = async (robotId) => {
        try {
            await axios.put(`${URL}/TaskLog/cancel/robot/${robotId}`, {}, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            console.log(`Stopped running tasks for robot ${robotId}`);
        } catch (error) {
            console.error("Error stopping running tasks:", error);
            throw error;
        }
    };

    const createTaskLog = async (robotId, taskType, parameters) => {
        try {
            const taskLogData = {
                taskTemplateId: 1, // You might want to map task types to template IDs
                robotIds: [robotId],
                parameters: parameters
            };

            const response = await axios.post(`${URL}/TaskLog`, taskLogData, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            return response.data.id;
        } catch (error) {
            console.error("Error creating TaskLog:", error);
            return null;
        }
    };

    const handleSendCommand = async (robotId, commandType, commandParams) => {
        if (!robotId || !commandType || !commandParams || !connectionRef.current) {
            return;
        }

        try {
            // 1. Stop running tasks
            await stopRunningTasksForRobot(robotId);

            // 2. Create task log
            const taskLogId = await createTaskLog(robotId, commandType, commandParams);

            // 3. Send command via SignalR
            await connectionRef.current.invoke("SendCommand", robotId, commandType, JSON.stringify(commandParams));

            // 4. Update local state
            const commandId = `cmd_${Date.now()}_${robotId}`;
            setRunningCommands(prev => ({
                ...prev,
                [robotId]: {
                    id: commandId,
                    taskLogId: taskLogId,
                    command: commandType,
                    parameters: commandParams,
                    startTime: new Date(),
                    status: "running"
                }
            }));

            // 5. Add to history
            setCommandHistory(prev => [{
                id: commandId,
                robotId: robotId,
                command: commandType,
                parameters: commandParams,
                timestamp: new Date(),
                status: "sent"
            }, ...prev.slice(0, 9)]);

            showFeedback(`Command sent to Robot ${robotId}`, "success");

        } catch (error) {
            console.error("Error sending command:", error);
            showFeedback(`Error sending command: ${error.message}`, "error");
        }
    };

    const handleStopCommand = async (robotId) => {
        try {
            await stopRunningTasksForRobot(robotId);

            setRunningCommands(prev => {
                const updated = { ...prev };
                delete updated[robotId];
                return updated;
            });

            showFeedback(`Command stopped for Robot ${robotId}`, "success");
        } catch (error) {
            console.error("Error stopping command:", error);
            showFeedback(`Error stopping command: ${error.message}`, "error");
        }
    };

    const getConnectionColor = () => {
        switch (connectionStatus) {
            case "connected":
                return "success";
            case "connecting":
            case "reconnecting":
                return "warning";
            case "error":
            case "disconnected":
                return "error";
            default:
                return "default";
        }
    };

    return (
        <Box className="p-24 h-full">
            {/* Connection Status */}
            <Paper
                elevation={2}
                sx={{
                    p: 2,
                    mb: 3,
                    bgcolor: getConnectionColor() === "success" ? "success.light" :
                             getConnectionColor() === "error" ? "error.light" : "warning.light",
                    color: getConnectionColor() === "success" ? "success.contrastText" :
                           getConnectionColor() === "error" ? "error.contrastText" : "warning.contrastText"
                }}
            >
                <Box className="flex items-center justify-center gap-8">
                    <Box
                        className="w-12 h-12 rounded-full"
                        sx={{
                            bgcolor: getConnectionColor() === "success" ? "success.main" :
                                    getConnectionColor() === "error" ? "error.main" : "warning.main",
                            animation: connectionStatus === "connecting" || connectionStatus === "reconnecting"
                                ? "pulse 2s infinite" : "none"
                        }}
                    />
                    <Typography variant="h6" className="font-bold">
                        Connection Status: {connectionStatus.toUpperCase()}
                    </Typography>
                </Box>
            </Paper>

            <Grid container spacing={3}>
                {/* VNC Viewer */}
                <Grid item xs={12}>
                    <Paper elevation={2} sx={{ p: 2 }}>
                        <Typography variant="h6" className="font-bold mb-16">
                            Robot View
                        </Typography>
                        <VncViewer url="wss://websocket.zerav.la" username="rs" password="123456789" />
                    </Paper>
                </Grid>

                {/* Command Panel */}
                <Grid item xs={12} lg={6}>
                    <CommandPanel
                        robots={robots}
                        selectedRobot={selectedRobot}
                        onRobotChange={setSelectedRobot}
                        onSendCommand={handleSendCommand}
                        connectionStatus={connectionStatus}
                        userId={userId}
                    />
                </Grid>

                {/* Command Status */}
                <Grid item xs={12} lg={6}>
                    <CommandStatus
                        runningCommands={runningCommands}
                        commandHistory={commandHistory}
                        onStopCommand={handleStopCommand}
                    />
                </Grid>
            </Grid>

            {/* Feedback Snackbar */}
            <Snackbar
                open={commandFeedback.open}
                autoHideDuration={5000}
                onClose={handleCloseFeedback}
                anchorOrigin={{ vertical: 'bottom', horizontal: 'right' }}
            >
                <Alert
                    onClose={handleCloseFeedback}
                    severity={commandFeedback.severity}
                    variant="filled"
                    sx={{ width: '100%' }}
                >
                    {commandFeedback.message}
                </Alert>
            </Snackbar>
        </Box>
    );
}

export default RealtimeConfigListImproved;
