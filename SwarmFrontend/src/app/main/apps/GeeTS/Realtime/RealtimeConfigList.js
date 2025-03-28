import { useState, useEffect } from "react";
import SendIcon from "@mui/icons-material/Send";
import { useParamList, useServiceList, useTopicList } from "rosreact";
import { InputAdornment, InputLabel, OutlinedInput, Select, MenuItem, ListSubheader } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import VncViewer from "./VncViewer";

const defaultTopics = ["/robot/:id/sensor_data", "/robot/:id/task/start", "/robot/:id/status"];

function RealtimeConfigList() {
    const [robots, setRobot] = useState([]);
    const [topics, setTopics] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState("");
    const [selectedTopic, setSelectedTopic] = useState("");
    const [command, setCommand] = useState("");
    // const [connection] = useState(() => singletonInstance.createConnectionBuilder());
    const [connectionStatus, setConnectionStatus] = useState("disconnected");

    // useEffect(() => {
    //     console.log("Initializing SceneManager...");
    //     let sceneManager;
    //     let connectionCheckInterval;
    //
    //     const initializeScene = () => {
    //         try {
    //             const token = jwtService.getAccessToken();
    //             // Use WSS protocol for direct WebSocket connection
    //             const wsUrl = "wss://robot.zerav.la/WebSocket/ws";
    //
    //             console.log("Initializing SceneManager with URL:", wsUrl);
    //
    //             sceneManager = new SceneManager({
    //                 elementId: "gz-scene",
    //                 websocketUrl: wsUrl,
    //                 // websocketKey: token,
    //                 // websocketHeaders: {
    //                 //     'Authorization': `Bearer ${token}`
    //                 // }
    //             });
    //
    //             // Monitor connection status
    //             const checkConnection = () => {
    //                 try {
    //                     const status = sceneManager.getConnectionStatus();
    //                     console.log("Current connection status:", status);
    //                     setConnectionStatus(status);
    //
    //                     if (status === "closed" || status === "error") {
    //                         console.log("Connection lost, attempting to reconnect...");
    //                         try {
    //                             sceneManager.connect(wsUrl);
    //                         } catch (error) {
    //                             console.error("Reconnection attempt failed:", error);
    //                             console.error("Error details:", error.message);
    //                         }
    //                     }
    //                 } catch (error) {
    //                     console.error("Error checking connection:", error);
    //                 }
    //             };
    //
    //             // Initial connection
    //             console.log("Attempting initial connection...");
    //             sceneManager.connect(wsUrl);
    //             console.log("SceneManager initialized");
    //
    //             // Check connection status less frequently (every 10 seconds)
    //             connectionCheckInterval = setInterval(checkConnection, 10000);
    //
    //             // Subscribe to connection status updates with error handling
    //             let subscription;
    //             try {
    //                 subscription = sceneManager.getConnectionStatusAsObservable().subscribe(
    //                     (isConnected) => {
    //                         console.log("Connection status changed:", isConnected ? "connected" : "disconnected");
    //                         setConnectionStatus(isConnected ? "connected" : "disconnected");
    //                     },
    //                     (error) => {
    //                         console.error("Connection observable error:", error);
    //                         setConnectionStatus("error");
    //                     },
    //                 );
    //             } catch (error) {
    //                 console.error("Error setting up connection observer:", error);
    //             }
    //
    //             return () => {
    //                 console.log("Cleaning up SceneManager...");
    //                 if (connectionCheckInterval) {
    //                     clearInterval(connectionCheckInterval);
    //                 }
    //                 if (subscription) {
    //                     try {
    //                         subscription.unsubscribe();
    //                     } catch (error) {
    //                         console.error("Error unsubscribing:", error);
    //                     }
    //                 }
    //                 if (sceneManager) {
    //                     try {
    //                         sceneManager.disconnect();
    //                         sceneManager.destroy();
    //                     } catch (error) {
    //                         console.error("Error during cleanup:", error);
    //                     }
    //                 }
    //             };
    //         } catch (error) {
    //             console.error("Error initializing SceneManager:", error);
    //             console.error("Stack trace:", error.stack);
    //             setConnectionStatus("error");
    //             return () => {
    //                 if (connectionCheckInterval) {
    //                     clearInterval(connectionCheckInterval);
    //                 }
    //             };
    //         }
    //     };
    //
    //     const cleanup = initializeScene();
    //     return cleanup;
    // }, []);

    // Add connection status indicator to the UI
    const getConnectionStatusColor = () => {
        switch (connectionStatus) {
            case "connected":
                return "bg-green-500";
            case "connecting":
                return "bg-yellow-500";
            case "error":
            case "closed":
            case "disconnected":
            default:
                return "bg-red-500";
        }
    };

    useEffect(() => {
        axios
            .get(`${URL}/Robots`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then((value) => {
                const { data } = value;
                setRobot(data);
                // Flatten the nested arrays to get a single array of topics
                const paths = data.flatMap((robot) =>
                    defaultTopics.map((topic) => topic.replace(":id", robot.id))
                );
                setTopics(paths);
                console.log("Topics updated:", paths);
            })
            .catch((error) => {
                console.error("Error fetching robots:", error);
            });
    }, []);

    function onSend(e) {
        console.log("SendCommand", selectedTopic, selectedRobot);
        // connection.send("SendCommand", selectedRobot, selectedTopic, command);
    }

    return (
        <div className="flex flex-1 flex-col items-center h-full">
            <div className="w-full p-2 flex items-center justify-between">
                <div className="flex items-center">
                    <div className={`w-3 h-3 rounded-full mr-2 ${getConnectionStatusColor()}`} />
                    <span className="text-sm">{connectionStatus}</span>
                </div>
            </div>
            <div className="p-2 flex ">
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
                                Robot {robot.id}
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
                        onChange={(e) => setSelectedTopic(e.target.value)}
                        className="w-full"
                    >
                        <ListSubheader>Sensors</ListSubheader>
                        {topics
                            .filter(
                                (topic) =>
                                    topic.includes("sensor") &&
                                    (selectedRobot === "all" || topic.includes(selectedRobot))
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
                                    (selectedRobot === "all" || topic.includes(selectedRobot))
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
                                    (selectedRobot === "all" || topic.includes(selectedRobot))
                            )
                            .map((topic) => (
                                <MenuItem key={topic} value={topic}>
                                    {topic}
                                </MenuItem>
                            ))}
                    </Select>
                </div>
                <div className="w-full">
                    <InputLabel htmlFor="outlined-adornment-password">Comando</InputLabel>
                    <OutlinedInput
                        id="outlined-adornment-password"
                        type="text"
                        className="w-full"
                        value={command}
                        onChange={(e) => setCommand(e.target.value)}
                        endAdornment={
                            <InputAdornment position="end">
                                <IconButton onClick={onSend} edge="end">
                                    <SendIcon />
                                </IconButton>
                            </InputAdornment>
                        }
                        label="Comando"
                    />
                </div>
            </div>
            <VncViewer url="wss://websocket.zerav.la" username="rs" password="123456789" />
        </div>
    );
}

const TopicListView = () => {
    const topicList = useTopicList();
    return (
        <>
            <p>{`${topicList.topics}`}</p>
            <p>{`${topicList.types}`}</p>
        </>
    );
};

const ServiceListView = () => {
    const list = useServiceList();
    return <p>{`${list}`}</p>;
};

const ParamListView = () => {
    const list = useParamList();
    return <p>{`${list}`}</p>;
};

export default RealtimeConfigList;
