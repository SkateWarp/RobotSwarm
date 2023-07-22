import Typography from "@mui/material/Typography";
import { useState, useEffect, Fragment } from "react";
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
            <RosConnection url="ws://127.0.0.1:9090" autoConnect>
                <PauseButton />
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
