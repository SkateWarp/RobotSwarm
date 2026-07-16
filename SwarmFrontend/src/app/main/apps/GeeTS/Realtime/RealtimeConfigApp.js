/* eslint-disable */
import withReducer from "app/store/withReducer";
import { useRef } from "react";
import SimulationWorkspace from "./SimulationWorkspace";
import reducer from "./store";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// Nota poner los hooks de estilo fuera del componente para evitar renderizado extra cuando haya busqueda
const Root = useGeneralAppStyle();

function RealtimeConfigApp() {
    const pageLayout = useRef(null);

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
                content={<SimulationWorkspace />}
                ref={pageLayout}
                innerScroll
            />
        </>
    );
}

export default withReducer("realtimeConfigApp", reducer)(RealtimeConfigApp);
