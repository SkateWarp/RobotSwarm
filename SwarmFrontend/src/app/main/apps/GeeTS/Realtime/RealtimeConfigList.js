import { useState, useEffect } from "react";
import SendIcon from "@mui/icons-material/Send";
import {
    RosConnection,
    TopicListProvider,
    ParamListProvider,
    ServiceListProvider,
    useParamList,
    useServiceList,
    useTopicList,
} from "rosreact";
import { InputAdornment, InputLabel, OutlinedInput } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import axios from "axios";
import PauseButton from "./buttons/PauseButton";
import GenericServiceButton from "./buttons/GenericServiceButton";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";

const defaultTopics = [

    "/robot/:id/sensor_data",
    "/robot/:id/task/start",
    "/robot/:id/status",

];

function RealtimeConfigList() {
    const [robots, setRobot] = useState([]);
    const [topics, setTopics] = useState([]);
    const [selectedTopic, setSelectedTopic] = useState("");
    const [command, setCommand] = useState("");
    const [trigger, setTrigger] = useState(false);
    const connection = singletonInstance.createConnectionBuilder();
    useEffect(() => {
        setTimeout(() => {
            setTrigger(!trigger);
        }, 3000);
    }, [trigger]);

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
                const paths = [data.map((robot) => defaultTopics.map((t) => t.replace(":id", robot.id)))];
                setTopics(paths);
                console.log(topics);
            });
    }, []);

    function onSend(e) {
        console.log("onSend", e.target.value, command);
        // connection.send();
    }

    return (
        <div className="flex flex-1 flex-col items-center h-full">
            <div className="p-2">
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
            <RosConnection url="https://robot.zerav.la/hubs/robot" autoConnect>
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
            </RosConnection>
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
