import { useState, useEffect } from "react";
import { useDispatch, useSelector } from "react-redux";
import {
    RosConnection,
    TopicListProvider,
    ParamListProvider,
    ServiceListProvider,
    useParamList,
    useServiceList,
    useTopicList,
} from "rosreact";
import PauseButton from "./buttons/PauseButton";
import GenericServiceButton from "./buttons/GenericServiceButton";

function RealtimeConfigList() {
    const dispatch = useDispatch();
    const [trigger, setTrigger] = useState(false);
    const searchText = useSelector(({ realtimeConfigApp }) => realtimeConfigApp.config.searchText);

    useEffect(() => {
        setTimeout(() => {
            setTrigger(!trigger);
        }, 3000);
    }, [trigger]);

    return (
        <div className="flex flex-1 items-center justify-center h-full">
            <RosConnection url="https://robot.zerav.la/hubs/robot" autoConnect >
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
