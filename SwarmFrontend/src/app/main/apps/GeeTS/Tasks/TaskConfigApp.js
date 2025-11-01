/* eslint-disable */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import TaskConfigDialog from "./TaskConfigDialog";
import TaskConfigList from "./TaskConfigList";
import reducer from "./store";
import { getTasks, openNewTaskConfigDialog } from "./store/taskConfigSlice";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// Nota poner los hooks de estilo fuera del componente para evitar renderizado extra cuando haya busqueda
const Root = useGeneralAppStyle();

function TaskConfigApp() {
    const dispatch = useDispatch();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getTasks());
    }, []);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader
                        pageLayout={pageLayout}
                        headerName="Tareas"
                        iconType="adjust"
                        hasSidebar={false}
                        actionButton={{
                            text: "CREAR",
                            onClick: () => dispatch(openNewTaskConfigDialog()),
                            icon: "add"
                        }}
                    />
                }
                content={<TaskConfigList />}
                ref={pageLayout}
                innerScroll
            />
            <TaskConfigDialog />
        </>
    );
}

export default withReducer("taskConfigApp", reducer)(TaskConfigApp);
