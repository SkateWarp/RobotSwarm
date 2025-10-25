import { useState, useEffect, useRef } from "react";
import SendIcon from "@mui/icons-material/Send";
import { InputAdornment, InputLabel, OutlinedInput, Select, MenuItem, ListSubheader } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import VncViewer from "./VncViewer";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
import useLoggedUserId from "../../../../shared-components/hooks/useLoggedUserId";

const defaultTopics = ["/robot/:id/sensor_data", "/robot/:id/task/start", "/robot/:id/status"];

function RealtimeConfigList() {
    const [robots, setRobot] = useState([]);
    const [topics, setTopics] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState("");
    const [selectedTopic, setSelectedTopic] = useState("");
    const [command, setCommand] = useState("");
    const [connectionStatus, setConnectionStatus] = useState("disconnected");
    const [commandFeedback, setCommandFeedback] = useState("");
    const [runningCommands, setRunningCommands] = useState({});
    const [commandHistory, setCommandHistory] = useState([]);

    const userId = useLoggedUserId();
    const connectionRef = useRef(null);
    const eventHandlerRef = useRef(null);


    useEffect(() => {
        // Setup SignalR connection
        setupSignalRConnection();

        // Fetch robots accessible to current user
        fetchUserRobots();

        // Load existing running TaskLogs
        loadRunningTaskLogs();

        // Cleanup on unmount
        return () => {
            cleanupConnection();
        };
    }, [userId]);

    const setupSignalRConnection = async () => {
        try {
            setConnectionStatus("connecting");

            // Create connection for current user (without group for general commands)
            connectionRef.current = singletonInstance.createConnectionBuilder();

            // Wait for connection to be established
            await singletonInstance.getConnectionPromise();

            // Register event handlers
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

        // Handle command execution feedback
        connectionRef.current.on("ExecuteCommand", (commandData) => {
            console.log("Command executed:", commandData);
            setCommandFeedback(`Command executed: ${commandData.command} at ${new Date(commandData.timestamp).toLocaleTimeString()}`);

            // Clear feedback after 5 seconds
            setTimeout(() => setCommandFeedback(""), 5000);
        });

        // Handle connection state changes
        connectionRef.current.onreconnecting(() => {
            setConnectionStatus("reconnecting");
        });

        connectionRef.current.onreconnected(() => {
            setConnectionStatus("connected");
        });

        connectionRef.current.onclose(() => {
            setConnectionStatus("disconnected");
        });
    };

    const fetchUserRobots = async () => {
        try {
            const response = await axios.get(`${URL}/Robots`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            const { data } = response;

            // Filter robots: user's own robots + public robots
            const accessibleRobots = data.filter(robot =>
                robot.accountId === userId || robot.isPublic
            );

            setRobot(accessibleRobots);

            // Generate topics for accessible robots only
            const paths = accessibleRobots.flatMap((robot) =>
                defaultTopics.map((topic) => topic.replace(":id", robot.id))
            );
            setTopics(paths);

        } catch (error) {
            console.error("Error fetching robots:", error);
        }
    };

    const loadRunningTaskLogs = async () => {
        try {
            const response = await axios.get(`${URL}/api/TaskLog`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            const taskLogs = response.data;

            // Find running TaskLogs for user's robots
            const runningTaskLogs = taskLogs.filter(task =>
                task.dateFinished === null &&
                task.dateCancelled === null &&
                task.robots?.some(robot => robot.accountId === userId)
            );

            // Convert running TaskLogs to running commands state
            const runningState = {};
            runningTaskLogs.forEach(task => {
                task.robots.forEach(robot => {
                    if (robot.accountId === userId) {
                        runningState[robot.id] = {
                            id: `task_${task.id}`,
                            taskLogId: task.id,
                            command: task.taskTemplate?.taskTypeDescription || "Unknown",
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
            // Remove event handlers
            if (eventHandlerRef.current) {
                connectionRef.current.off("ExecuteCommand", eventHandlerRef.current);
            }

            // Stop connection
            singletonInstance.stopConnection();
            connectionRef.current = null;
        }
    };

    const onSend = async (e) => {
        e.preventDefault();

        if (!selectedRobot || !selectedTopic || !command.trim() || !connectionRef.current) {
            return;
        }

        try {
            // Extract robot ID from selected robot
            const robotId = selectedRobot === "all" ? null : parseInt(selectedRobot);

            if (!robotId) {
                setCommandFeedback("Please select a specific robot");
                setTimeout(() => setCommandFeedback(""), 3000);
                return;
            }

            // Extract topic type from selected topic
            let topicType = "unknown";
            if (selectedTopic.includes("sensor_data")) {
                topicType = "sensor_data";
            } else if (selectedTopic.includes("task")) {
                topicType = "task";
            } else if (selectedTopic.includes("status")) {
                topicType = "status";
            }

            // Prepare command parameters based on topic type
            let commandParams = command;

            // For sensor_data, extract JSON from command
            if (topicType === "sensor_data") {
                const jsonMatch = command.match(/data:\s*(\{.*\})/);
                if (jsonMatch) {
                    try {
                        commandParams = JSON.parse(jsonMatch[1]);
                    } catch (parseError) {
                        console.error("Error parsing sensor data JSON:", parseError);
                        setCommandFeedback("Invalid JSON format in sensor data");
                        setTimeout(() => setCommandFeedback(""), 3000);
                        return;
                    }
                }
            }
            // For task, extract JSON from command
            else if (topicType === "task") {
                const jsonMatch = command.match(/data:\s*(\{.*\})/);
                if (jsonMatch) {
                    try {
                        commandParams = JSON.parse(jsonMatch[1]);
                    } catch (parseError) {
                        console.error("Error parsing task JSON:", parseError);
                        setCommandFeedback("Invalid JSON format in task data");
                        setTimeout(() => setCommandFeedback(""), 3000);
                        return;
                    }
                }
            }
            // For status, extract string from command
            else if (topicType === "status") {
                const stringMatch = command.match(/data:\s*['"](.*)['"]/);
                if (stringMatch) {
                    commandParams = stringMatch[1];
                }
            }

            // 1. Check for running TaskLogs and stop them
            await stopRunningTasksForRobot(robotId);

            // 2. Create new TaskLog for this command
            const taskLogId = await createTaskLog(robotId, topicType, commandParams);

            // 3. Send command via SignalR
            await connectionRef.current.invoke("SendCommand", robotId, topicType, JSON.stringify(commandParams));

            // 4. Update local state to track running command
            const commandId = `cmd_${Date.now()}_${robotId}`;
            setRunningCommands(prev => ({
                ...prev,
                [robotId]: {
                    id: commandId,
                    taskLogId: taskLogId,
                    command: topicType,
                    parameters: commandParams,
                    startTime: new Date(),
                    status: "running"
                }
            }));

            // 5. Add to command history
            setCommandHistory(prev => [{
                id: commandId,
                robotId: robotId,
                command: topicType,
                parameters: commandParams,
                timestamp: new Date(),
                status: "sent"
            }, ...prev.slice(0, 9)]); // Keep last 10 commands

            setCommandFeedback(`Command sent to robot ${robotId} for topic ${topicType}`);
            setTimeout(() => setCommandFeedback(""), 3000);

            console.log(`Command sent: Robot=${robotId}, Topic=${topicType}, Command=${JSON.stringify(commandParams)}`);

        } catch (error) {
            console.error("Error sending command:", error);
            setCommandFeedback(`Error sending command: ${error.message}`);
            setTimeout(() => setCommandFeedback(""), 5000);
        }
    };

    const stopRunningTasksForRobot = async (robotId) => {
        try {
            // Get current TaskLogs to find running ones for this robot
            const response = await axios.get(`${URL}/api/TaskLog`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            });

            const taskLogs = response.data;
            const runningTasks = taskLogs.filter(task =>
                task.dateFinished === null &&
                task.dateCancelled === null &&
                task.robots?.some(robot => robot.id === robotId)
            );

            // Cancel all running tasks for this robot
            for (const task of runningTasks) {
                await axios.put(`${URL}/api/TaskLog/cancel/${robotId}`, {}, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${jwtService.getAccessToken()}`,
                    },
                });
            }

            if (runningTasks.length > 0) {
                console.log(`Stopped ${runningTasks.length} running task(s) for robot ${robotId}`);
            }

        } catch (error) {
            console.error("Error stopping running tasks:", error);
        }
    };

    const createTaskLog = async (robotId, topicType, commandParams) => {
        try {
            // Map topic type to TaskTemplate
            let taskTemplateId = 1; // Default task template

            // You might want to create specific task templates for each command type
            // For now, we'll use a generic one
            const taskLogData = {
                taskTemplateId: taskTemplateId,
                robotIds: [robotId],
                parameters: commandParams
            };

            const response = await axios.post(`${URL}/api/TaskLog`, taskLogData, {
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

    const stopCommand = async (robotId) => {
        try {
            // Stop the running command by cancelling its TaskLog
            await stopRunningTasksForRobot(robotId);

            // Update local state
            setRunningCommands(prev => {
                const updated = { ...prev };
                if (updated[robotId]) {
                    updated[robotId] = { ...updated[robotId], status: "stopped" };
                }
                return updated;
            });

            setCommandFeedback(`Command stopped for robot ${robotId}`);
            setTimeout(() => setCommandFeedback(""), 3000);

        } catch (error) {
            console.error("Error stopping command:", error);
            setCommandFeedback(`Error stopping command: ${error.message}`);
            setTimeout(() => setCommandFeedback(""), 5000);
        }
    };

    return (
        <div className="p-8 flex flex-1 flex-col items-center h-full">
            {/* Connection Status */}
            <div className="mb-4 text-center">
                <div className={`inline-flex items-center px-3 py-1 rounded-full text-sm font-medium ${
                    connectionStatus === "connected"
                        ? "bg-green-100 text-green-800"
                        : connectionStatus === "connecting"
                        ? "bg-yellow-100 text-yellow-800"
                        : connectionStatus === "reconnecting"
                        ? "bg-blue-100 text-blue-800"
                        : "bg-red-100 text-red-800"
                }`}>
                    <div className={`w-2 h-2 rounded-full mr-2 ${
                        connectionStatus === "connected"
                            ? "bg-green-500"
                            : connectionStatus === "connecting" || connectionStatus === "reconnecting"
                            ? "bg-yellow-500 animate-pulse"
                            : "bg-red-500"
                    }`}></div>
                    Connection: {connectionStatus}
                </div>
            </div>

            <div className="pb-32">
                <VncViewer url="wss://websocket.zerav.la" username="rs" password="123456789" />
            </div>
            <div className="p-2 pb-8 flex flex-col">
                {/* Running Commands Section */}
                {Object.keys(runningCommands).length > 0 && (
                    <div className="mb-6 p-4 bg-blue-50 border border-blue-200 rounded-lg">
                        <h3 className="text-lg font-semibold text-blue-800 mb-3">Running Commands</h3>
                        <div className="space-y-2">
                            {Object.entries(runningCommands).map(([robotId, commandInfo]) => (
                                <div key={robotId} className="flex items-center justify-between p-3 bg-white rounded border">
                                    <div className="flex-1">
                                        <div className="flex items-center space-x-2">
                                            <span className="font-medium">Robot {robotId}</span>
                                            <span className="px-2 py-1 bg-green-100 text-green-800 text-xs rounded-full">
                                                {commandInfo.command}
                                            </span>
                                            <span className="text-sm text-gray-600">
                                                Started: {commandInfo.startTime.toLocaleTimeString()}
                                            </span>
                                        </div>
                                    </div>
                                    <button
                                        onClick={() => stopCommand(parseInt(robotId))}
                                        className="px-3 py-1 bg-red-500 text-white text-sm rounded hover:bg-red-600 transition-colors"
                                    >
                                        Stop
                                    </button>
                                </div>
                            ))}
                        </div>
                    </div>
                )}

                {/* Command History Section */}
                {commandHistory.length > 0 && (
                    <div className="mb-6 p-4 bg-gray-50 border border-gray-200 rounded-lg">
                        <h3 className="text-lg font-semibold text-gray-800 mb-3">Recent Commands</h3>
                        <div className="space-y-2 max-h-40 overflow-y-auto">
                            {commandHistory.map((historyItem) => (
                                <div key={historyItem.id} className="flex items-center space-x-2 p-2 bg-white rounded border">
                                    <span className="font-medium text-sm">Robot {historyItem.robotId}</span>
                                    <span className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded-full">
                                        {historyItem.command}
                                    </span>
                                    <span className="text-xs text-gray-600">
                                        {historyItem.timestamp.toLocaleTimeString()}
                                    </span>
                                    <span className={`px-2 py-1 text-xs rounded-full ${
                                        historyItem.status === "sent"
                                            ? "bg-green-100 text-green-800"
                                            : "bg-gray-100 text-gray-800"
                                    }`}>
                                        {historyItem.status}
                                    </span>
                                </div>
                            ))}
                        </div>
                    </div>
                )}

                <div className="flex flex-col mb-4">
                    <div className="flex mb-4">
                        <div className="w-full mr-2">
                            <InputLabel id="robot-select-label">Robot</InputLabel>
                            <Select
                                labelId="robot-select-label"
                                id="robot-select"
                                value={selectedRobot}
                                onChange={(e) => setSelectedRobot(e.target.value)}
                                className="w-full"
                            >
                                <MenuItem value="all">All Robots</MenuItem>
                                {robots.map((robot) => (
                                    <MenuItem key={robot.id} value={robot.id}>
                                        Robot {robot.id} {robot.accountId === userId ? "(My Robot)" : robot.isPublic ? "(Public)" : "(Private)"}
                                    </MenuItem>
                                ))}
                            </Select>
                        </div>
                        <div className="w-full mr-2">
                            <InputLabel id="topic-select-label">Topic</InputLabel>
                            <Select
                                labelId="topic-select-label"
                                id="topic-select"
                                value={selectedTopic}
                                onChange={(e) => {
                                    const topic = e.target.value;
                                    setSelectedTopic(topic);
                                    if (topic.includes("sensor_data")) {
                                        setCommand(`data: '{"left_ticks": 21.0, "right_ticks": 20.0, "left_diff": 19.0, "right_diff": 18.0, "left_dist": 17.0, "right_dist": 16.0, "timestep": 15.0, "left_speed": 14.0, "right_speed": 13.0, "left_speed_filtered": 12.0, "right_speed_filtered": 11.0, "name": "Speed"}'`);
                                    } else if (topic.includes("task")) {
                                        setCommand(`data: '{"taskType": "transporte", "parameters": {"sensorName": "speed", "value": 23.5}}'`);
                                    } else if (topic.includes("status")) {
                                        setCommand("data: 'Working'");
                                    } else {
                                        setCommand("");
                                    }
                                }}
                                className="w-full"
                            >
                                <ListSubheader>Sensors</ListSubheader>
                                {topics
                                    .filter(
                                        (topic) =>
                                            topic.includes("sensor") &&
                                            (selectedRobot === "all" || topic.includes(selectedRobot)),
                                    )
                                    .map((topic) => (
                                        <MenuItem key={topic} value={topic}>
                                            {topic}
                                        </MenuItem>
                                    ))}
                                <ListSubheader>Tasks</ListSubheader>
                                {topics
                                    .filter(
                                        (topic) =>
                                            topic.includes("task") &&
                                            (selectedRobot === "all" || topic.includes(selectedRobot)),
                                    )
                                    .map((topic) => (
                                        <MenuItem key={topic} value={topic}>
                                            {topic}
                                        </MenuItem>
                                    ))}
                                <ListSubheader>Status</ListSubheader>
                                {topics
                                    .filter(
                                        (topic) =>
                                            topic.includes("status") &&
                                            (selectedRobot === "all" || topic.includes(selectedRobot)),
                                    )
                                    .map((topic) => (
                                        <MenuItem key={topic} value={topic}>
                                            {topic}
                                        </MenuItem>
                                    ))}
                            </Select>
                        </div>
                        <IconButton
                            onClick={onSend}
                            disabled={!selectedRobot || !selectedTopic || !command.trim() || connectionStatus !== "connected"}
                            className="p-4  h-full"
                        >
                            <SendIcon color={connectionStatus === "connected" ? "green" : "disabled"} />
                        </IconButton>

                    </div>
                    <div className="w-full">
                        <InputLabel htmlFor="command-input">Comando</InputLabel>
                        <OutlinedInput
                            id="command-input"
                            type="text"
                            multiline
                            rows={4}
                            className="w-full"
                            value={command}
                            onChange={(e) => setCommand(e.target.value)}
                            label="Comando"
                        />
                    </div>

                    {/* Command Feedback */}
                    {commandFeedback && (
                        <div className={`mt-4 p-3 rounded-md text-sm font-medium ${
                            commandFeedback.includes("Error") || commandFeedback.includes("Invalid")
                                ? "bg-red-100 text-red-800 border border-red-200"
                                : commandFeedback.includes("sent") || commandFeedback.includes("executed")
                                ? "bg-green-100 text-green-800 border border-green-200"
                                : "bg-blue-100 text-blue-800 border border-blue-200"
                        }`}>
                            {commandFeedback}
                        </div>
                    )}
                </div>

            </div>
        </div>
    );
}


export default RealtimeConfigList;
