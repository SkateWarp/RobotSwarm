/* eslint-disable react-hooks/exhaustive-deps */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import reducer from "./store";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import { getLeafTypes, openNewLeafTypesConfigDialog } from "./store/leafTypeConfigSlice";
import LeafTypesConfigList from "./LeafTypeConfigList";
import LeafTypesConfigDialog from "./LeafTypeConfigDialog";
import SimpleSidebarContent from "../../../../shared-components/SimpleSidebarContent";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function LeafTypesConfigApp() {
    const dispatch = useDispatch();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getLeafTypes());
    }, []);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader pageLayout={pageLayout} headerName="Tareas" iconType="add_task" />
                }
                content={<LeafTypesConfigList />}
                leftSidebarContent={
                    <SimpleSidebarContent
                        buttonText="Crear"
                        openDialogFunction={openNewLeafTypesConfigDialog}
                    />
                }
                sidebarInner
                ref={pageLayout}
                innerScroll
            />
            <LeafTypesConfigDialog />
        </>
    );
}

export default withReducer("leafTypesConfigApp", reducer)(LeafTypesConfigApp);
