/* eslint-disable */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import RealtimeConfigDialog from "./RealtimeConfigDialog";
import RealtimeConfigList from "./RealtimeConfigList";
import RealtimeConfigSidebarContent from "./RealtimeConfigSidebarContent";
import reducer from "./store";
import { getTasks } from "./store/realtimeConfigSlice";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// Nota poner los hooks de estilo fuera del componente para evitar renderizado extra cuando haya busqueda
const Root = useGeneralAppStyle();

function RealtimeConfigApp() {
    const dispatch = useDispatch();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getTasks());
    }, []);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader pageLayout={pageLayout} headerName="Realtime" iconType="adjust" />
                }
                content={<RealtimeConfigList />}
                // leftSidebarContent={<RealtimeConfigSidebarContent />}
                sidebarInner
                ref={pageLayout}
                innerScroll
            />
            {/* <RealtimeConfigDialog /> */}
        </>
    );
}

export default withReducer("realtimeConfigApp", reducer)(RealtimeConfigApp);
