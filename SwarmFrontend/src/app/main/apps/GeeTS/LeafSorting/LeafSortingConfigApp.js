/* eslint-disable react-hooks/exhaustive-deps */
import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import reducer from "./store";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import { getLeafSorting, openNewLeafSortingConfigDialog } from "./store/leafSortingConfigSlice";
import LeafSortingConfigList from "./LeafSortingConfigList";
import LeafSortingConfigDialog from "./LeafSortingConfigDialog";
import SimpleSidebarContent from "../../../../shared-components/SimpleSidebarContent";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function LeafSortingConfigApp() {
    const dispatch = useDispatch();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getLeafSorting());
    }, []);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader pageLayout={pageLayout} headerName="Robots" iconType="settings" />
                }
                content={<LeafSortingConfigList />}
                leftSidebarContent={
                    <SimpleSidebarContent
                        buttonText="Crear"
                        openDialogFunction={openNewLeafSortingConfigDialog}
                    />
                }
                sidebarInner
                ref={pageLayout}
                innerScroll
            />
            <LeafSortingConfigDialog />
        </>
    );
}

export default withReducer("leafSortingConfigApp", reducer)(LeafSortingConfigApp);
