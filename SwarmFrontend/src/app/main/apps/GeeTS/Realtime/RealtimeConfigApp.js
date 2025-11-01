/* eslint-disable */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import RealtimeConfigListImproved from "./RealtimeConfigListImproved";
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
                    <SimpleGeneralHeader
                        pageLayout={pageLayout}
                        headerName="Control en Tiempo Real"
                        iconType="settings_remote"
                        hasSidebar={false}
                    />
                }
                content={<RealtimeConfigListImproved />}
                ref={pageLayout}
                innerScroll
            />
        </>
    );
}

export default withReducer("realtimeConfigApp", reducer)(RealtimeConfigApp);
