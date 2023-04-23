import withReducer from "app/store/withReducer";
import { useRef } from "react";
import reducer from "./store";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import OperatorProductionList from "./OperatorProductionList";
import OperatorProductionSidebarContent from "./OperatorProductionSidebarContent";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function OperatorProductionApp() {
    const pageLayout = useRef(null);

    return (
        <Root
            header={
                <SimpleGeneralHeader
                    pageLayout={pageLayout}
                    headerName="Producción de Operadores"
                    iconType="account_box"
                />
            }
            content={<OperatorProductionList />}
            leftSidebarContent={<OperatorProductionSidebarContent />}
            sidebarInner
            ref={pageLayout}
            innerScroll
        />
    );
}

export default withReducer("operatorProductionApp", reducer)(OperatorProductionApp);
