/* eslint-disable react-hooks/exhaustive-deps */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import { useNavigate } from "react-router-dom";
import reducer from "./store";
import useGeneralAppStyle from "app/shared-components/hooks/useGeneralAppStyle";
import { getTaskLogs } from "./store/taskLogSlice";
import TaskLogList from "./TaskLogList";
import TaskLogDialog from "./TaskLogDialog";
import SimpleGeneralHeader from "app/shared-components/SimpleGeneralHeader";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function TaskLogApp() {
    const dispatch = useDispatch();
    const navigate = useNavigate();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getTaskLogs());
    }, []);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader
                        pageLayout={pageLayout}
                        headerName="Task Logs"
                        iconType="history"
                        hasSidebar={false}
                        actionButton={{
                            text: "CREAR TAREA",
                            onClick: () => navigate("/apps/GTS/realtime"),
                            icon: "add"
                        }}
                    />
                }
                content={<TaskLogList />}
                ref={pageLayout}
                innerScroll
            />
            <TaskLogDialog />
        </>
    );
}

export default withReducer("taskLogApp", reducer)(TaskLogApp);
