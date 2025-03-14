import { useState, useEffect } from "react";
import SendIcon from "@mui/icons-material/Send";
import {
    useParamList,
    useServiceList,
    useTopicList,
} from "rosreact";
import { InputAdornment, InputLabel, OutlinedInput, Select, MenuItem, ListSubheader, } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
const defaultTopics = [

    "/robot/:id/sensor_data",
    "/robot/:id/task/start",
    "/robot/:id/status",

];

function RealtimeConfigList() {
    const [robots, setRobot] = useState([]);
    const [topics, setTopics] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState("");
    const [selectedTopic, setSelectedTopic] = useState("");
    const [command, setCommand] = useState("");
    const [connection] = useState(() => singletonInstance.createConnectionBuilder());



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
                const paths = data.flatMap(robot =>
                    defaultTopics.map(topic => topic.replace(":id", robot.id))
                );
                setTopics(paths);
                console.log("Topics updated:", paths);
            })
            .catch(error => {
                console.error("Error fetching robots:", error);
            });
    }, []);

    function onSend(e) {
        console.log("SendCommand", selectedTopic, selectedRobot);
        connection.send("SendCommand", selectedRobot, selectedTopic, command);
    }

    return (
        <div className="flex flex-1 flex-col items-center h-full">
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
                            .filter(topic => topic.includes('sensor') && (selectedRobot === 'all' || topic.includes(selectedRobot)))
                            .map((topic) => (
                                <MenuItem key={topic} value={topic}>
                                    {topic}
                                </MenuItem>
                            ))}
                        <ListSubheader>Tasks</ListSubheader>
                        {topics
                            .filter(topic => topic.includes('task') && (selectedRobot === 'all' || topic.includes(selectedRobot)))
                            .map((topic) => (
                                <MenuItem key={topic} value={topic}>
                                    {topic}
                                </MenuItem>
                            ))}
                        <ListSubheader>Status</ListSubheader>
                        {topics
                            .filter(topic => topic.includes('status') && (selectedRobot === 'all' || topic.includes(selectedRobot)))
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
            {/* <RosConnection url="https://robot.zerav.la/hubs/robot" autoConnect>
                <PauseButton />
                <GenericServiceButton topicName="/gazebo/reset_simulation" text="Reset Simulation" />
                <GenericServiceButton topicName="/gazebo/reset_world" text="Reset World" />
                <GenericServiceButton
                    topicName="/gazebo/delete_model"
                    text="Delete model - no funciona - hay que enviarle las arenas"
                    serviceType="deleteModel"
                />
                <GenericServiceButton
                    topicName="/gazebo/spawn_urdf_model"
                    text="Commi arena - no funciona - hay que enviarle un path"
                    serviceType="spawnModel"
                />

                <GenericServiceButton
                    topicName="/gazebo/delete_model"
                    text="Delete model - no funciona - hay que enviarle los hero"
                    serviceType="deleteModel"
                />
                <TopicListProvider
                    trigger={trigger}
                    failedCallback={(e) => {
                        console.log(e);
                    }}
                >
                    <TopicListView />
                </TopicListProvider>

                <ServiceListProvider
                    trigger={trigger}
                    failedCallback={(e) => {
                        console.log(e);
                    }}
                >
                    <ServiceListView />
                </ServiceListProvider>

                <ParamListProvider
                    trigger={trigger}
                    failedCallback={(e) => {
                        console.log(e);
                    }}
                >
                    <ParamListView />
                </ParamListProvider>
            </RosConnection> */}
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
