import { useState, useEffect, useRef } from "react";
import {
    Box,
    Grid,
    Paper,
    Typography,
    Alert,
    Snackbar,
    Tabs,
    Tab
} from "@mui/material";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import VncViewer from "./VncViewer";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
import useLoggedUserId from "../../../../shared-components/hooks/useLoggedUserId";
import CommandPanel from "./CommandPanel";
import CommandStatus from "./CommandStatus";
import SwarmControlPanel from "./SwarmControlPanel";
import swarmService from "../../../../../services/SwarmService";

function RealtimeConfigListImproved() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState("");
    const [connectionStatus, setConnectionStatus] = useState("disconnected");
    const [commandFeedback, setCommandFeedback] = useState({ open: false, message: "", severity: "info" });
    const [runningCommands, setRunningCommands] = useState({});
    const [commandHistory, setCommandHistory] = useState([]);
    const [rightPanelTab, setRightPanelTab] = useState(0);

    // Estado del enjambre (swarm)
    const [swarmConnected, setSwarmConnected] = useState(false);
    const [rosConnected, setRosConnected] = useState(false);
    const [swarmStatus, setSwarmStatus] = useState(null);

    const userId = useLoggedUserId();
    const connectionRef = useRef(null);

    useEffect(() => {
        setupSignalRConnection();
        fetchUserRobots();
        loadRunningTaskLogs();
        setupSwarmService();

        return () => {
            cleanupConnection();
            swarmService.disconnect();
        };
    }, [userId]);

    // Configurar servicio de enjambre
    const setupSwarmService = async () => {
        swarmService.setStatusCallback((status) => {
            setSwarmStatus(status);
        });
        swarmService.setConnectionCallback((connected) => {
            setSwarmConnected(connected);
        });
        swarmService.setRosConnectionCallback((connected) => {
            setRosConnected(connected);
        });
        swarmService.setErrorCallback((errorMsg) => {
            showFeedback(errorMsg, "error");
        });

        try {
            await swarmService.connect();
        } catch (error) {
            console.error("Error al conectar servicio de enjambre:", error);
        }
    };

    const setupSignalRConnection = async () => {
        try {
            setConnectionStatus("connecting");
            connectionRef.current = singletonInstance.createConnectionBuilder();
            await singletonInstance.getConnectionPromise();
            setupEventHandlers();
            setConnectionStatus("connected");
            console.log("Conexión SignalR establecida correctamente");
        } catch (error) {
            console.error("Error al establecer conexión SignalR:", error);
            setConnectionStatus("error");
        }
    };

    const setupEventHandlers = () => {
        if (!connectionRef.current) return;

        connectionRef.current.on("ExecuteCommand", (commandData) => {
            console.log("Comando ejecutado:", commandData);
            showFeedback(`Comando ejecutado: ${commandData.command}`, "success");
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
            console.error("Error al obtener robots:", error);
            showFeedback("Error al obtener robots", "error");
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
                            command: task.taskTemplate?.name || "Desconocido",
                            parameters: task.parameters,
                            startTime: new Date(task.dateCreated),
                            status: "running"
                        };
                    }
                });
            });

            setRunningCommands(runningState);
        } catch (error) {
            console.error("Error al cargar tareas activas:", error);
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

            console.log(`Tareas detenidas para robot ${robotId}`);
        } catch (error) {
            console.error("Error al detener tareas:", error);
            throw error;
        }
    };

    const createTaskLog = async (robotId, taskType, parameters) => {
        try {
            const taskLogData = {
                taskTemplateId: 1,
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
            console.error("Error al crear registro de tarea:", error);
            return null;
        }
    };

    const handleSendCommand = async (robotId, commandType, commandParams) => {
        if (!robotId || !commandType || !commandParams || !connectionRef.current) {
            return;
        }

        try {
            if (commandType === "sensor_data") {
                const sensorName = commandParams.sensorType || "Speed";
                const sensorFields = { ...commandParams };
                delete sensorFields.name;
                delete sensorFields.sensorType;

                await connectionRef.current.invoke(
                    "HandleSensorReadingFromClient",
                    robotId,
                    sensorName,
                    sensorFields
                );

                const commandId = `sensor_${Date.now()}_${robotId}`;
                setCommandHistory(prev => [{
                    id: commandId,
                    robotId: robotId,
                    command: commandType,
                    parameters: commandParams,
                    timestamp: new Date(),
                    status: "sent"
                }, ...prev.slice(0, 9)]);

                showFeedback(`Datos de sensor enviados al Robot ${robotId}`, "success");
                return;
            }

            await stopRunningTasksForRobot(robotId);
            const taskLogId = await createTaskLog(robotId, commandType, commandParams);
            await connectionRef.current.invoke("SendCommand", robotId, commandType, JSON.stringify(commandParams));

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

            setCommandHistory(prev => [{
                id: commandId,
                robotId: robotId,
                command: commandType,
                parameters: commandParams,
                timestamp: new Date(),
                status: "sent"
            }, ...prev.slice(0, 9)]);

            showFeedback(`Comando enviado al Robot ${robotId}`, "success");

        } catch (error) {
            console.error("Error al enviar comando:", error);
            showFeedback(`Error al enviar comando: ${error.message}`, "error");
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

            showFeedback(`Comando detenido para Robot ${robotId}`, "success");
        } catch (error) {
            console.error("Error al detener comando:", error);
            showFeedback(`Error al detener comando: ${error.message}`, "error");
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

    const getConnectionLabel = () => {
        switch (connectionStatus) {
            case "connected": return "CONECTADO";
            case "connecting": return "CONECTANDO";
            case "reconnecting": return "RECONECTANDO";
            case "error": return "ERROR";
            case "disconnected": return "DESCONECTADO";
            default: return connectionStatus.toUpperCase();
        }
    };

    return (
        <Box className="p-24 h-full">
            {/* Estado de Conexión */}
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
                        Estado de Conexión: {getConnectionLabel()}
                    </Typography>
                </Box>
            </Paper>

            <Grid container spacing={3}>
                {/* Visor VNC - mitad izquierda */}
                <Grid item xs={12} lg={6}>
                    <Paper elevation={2} sx={{ p: 2, height: '100%' }}>
                        <Typography variant="h6" className="font-bold mb-16">
                            Vista de Robots
                        </Typography>
                        <VncViewer url="wss://websocket.zerav.la?aneyelo=gei" username="rs" password="123456789" />
                    </Paper>
                </Grid>

                {/* Panel derecho - mitad derecha con pestañas */}
                <Grid item xs={12} lg={6}>
                    <Paper elevation={2} sx={{ p: 2, height: '100%' }}>
                        <Tabs
                            value={rightPanelTab}
                            onChange={(e, newValue) => setRightPanelTab(newValue)}
                            variant="fullWidth"
                            sx={{ mb: 2 }}
                        >
                            <Tab label="Control del Enjambre" />
                            <Tab label="Enviar Comando" />
                        </Tabs>

                        {/* Pestaña: Control del Enjambre */}
                        {rightPanelTab === 0 && (
                            <Box sx={{ maxHeight: '70vh', overflow: 'auto' }}>
                                <SwarmControlPanel
                                    robots={robots}
                                    userId={userId}
                                    swarmService={swarmService}
                                    isConnected={swarmConnected}
                                    isRosConnected={rosConnected}
                                    swarmStatus={swarmStatus}
                                    onError={(msg) => showFeedback(msg, "error")}
                                />
                            </Box>
                        )}

                        {/* Pestaña: Enviar Comando */}
                        {rightPanelTab === 1 && (
                            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 3, maxHeight: '70vh', overflow: 'auto' }}>
                                <CommandPanel
                                    robots={robots}
                                    selectedRobot={selectedRobot}
                                    onRobotChange={setSelectedRobot}
                                    onSendCommand={handleSendCommand}
                                    connectionStatus={connectionStatus}
                                    userId={userId}
                                />
                                <CommandStatus
                                    runningCommands={runningCommands}
                                    commandHistory={commandHistory}
                                    onStopCommand={handleStopCommand}
                                />
                            </Box>
                        )}
                    </Paper>
                </Grid>
            </Grid>

            {/* Snackbar de Retroalimentación */}
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
