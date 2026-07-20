import withReducer from "app/store/withReducer";
import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import reducer from "./store";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import { getTaskTemplates } from "./store/leafTypeConfigSlice";
import LeafTypesConfigList from "./LeafTypeConfigList";
import LeafTypesConfigDialog from "./LeafTypeConfigDialog";
import SimpleGeneralHeader from "../../../../shared-components/SimpleGeneralHeader";

// The shared helper builds a styled component; it does not use React state.
// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function LeafTypesConfigApp() {
    const dispatch = useDispatch();
    const pageLayout = useRef(null);

    useEffect(() => {
        dispatch(getTaskTemplates());
    }, [dispatch]);

    return (
        <>
            <Root
                header={
                    <SimpleGeneralHeader
                        pageLayout={pageLayout}
                        headerName="PLANTILLAS DE TAREAS"
                        iconType="add_task"
                        hasSidebar={false}
                    />
                }
                content={<LeafTypesConfigList />}
                ref={pageLayout}
                innerScroll
            />
            <LeafTypesConfigDialog />
        </>
    );
}

export default withReducer("leafTypesConfigApp", reducer)(LeafTypesConfigApp);
