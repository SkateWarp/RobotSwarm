import withReducer from "app/store/withReducer";
import { useRef } from "react";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";
import SwarmControlPanel from "../../../../../components/SwarmControlPanel";

const Root = useGeneralAppStyle();

function SwarmControlApp() {
    const pageLayout = useRef(null);

    return (
        <Root
            header={
                <SimpleGeneralHeader
                    pageLayout={pageLayout}
                    headerName="Swarm Control"
                    iconType="smart_toy"
                    hasSidebar={false}
                />
            }
            content={<SwarmControlPanel />}
            ref={pageLayout}
            innerScroll
        />
    );
}

export default SwarmControlApp;
