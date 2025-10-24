import { useState, useEffect } from "react";
import SendIcon from "@mui/icons-material/Send";
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
                    defaultTopics.map((topic) => topic.replace(":id", robot.id)),
                );
                setTopics(paths);
            })
            .catch((error) => {
                console.error("Error fetching robots:", error);
            });
    }, []);

    function onSend(e) {
        // connection.send("SendCommand", selectedRobot, selectedTopic, command);
    }

    return (
        <div className="p-8 flex flex-1 flex-col items-center h-full">
            <div className="pb-32">
                <VncViewer url="wss://websocket.zerav.la" username="rs" password="123456789" />
            </div>
            <div className="p-2 pb-8 flex flex-col">
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
                            disabled={!selectedRobot || !selectedTopic || !command.trim()}
                            className="p-4  h-full"
                        >
                            <SendIcon color="green" />
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
                </div>

            </div>
        </div>
    );
}


export default RealtimeConfigList;
