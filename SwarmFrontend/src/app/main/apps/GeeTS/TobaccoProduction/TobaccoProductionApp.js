import withReducer from "app/store/withReducer";
import { useRef } from "react";
import reducer from "./store";
import TobaccoProductionList from "./TobaccoProductionList";
import TobaccoProductionSidebarContent from "./TobaccoProductionSidebarContent";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function TobaccoProductionApp() {
    const pageLayout = useRef(null);

    return (
        <Root
            header={
                <SimpleGeneralHeader
                    pageLayout={pageLayout}
                    headerName="Producción de Hojas"
                    iconType="today"
                />
            }
            content={<TobaccoProductionList />}
            leftSidebarContent={<TobaccoProductionSidebarContent />}
            sidebarInner
            ref={pageLayout}
            innerScroll
        />
    );
}

export default withReducer("tobaccoProductionApp", reducer)(TobaccoProductionApp);
