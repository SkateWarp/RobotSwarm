import withReducer from "app/store/withReducer";
import { useRef } from "react";
import { useSelector } from "react-redux";
import reducer from "./store";
import MachineProductionList from "./MachineProductionList";
import MachineProductionSidebarContent from "./MachineProductionSidebarContent";
import useGeneralAppStyle from "../../../../shared-components/hooks/useGeneralAppStyle";
import GeneralHeader from "../../../../shared-components/GeneralHeader";
import { setMachineProductionSearchText } from "./store/machineProductionSlice";

// eslint-disable-next-line react-hooks/rules-of-hooks
const Root = useGeneralAppStyle();

function MachineProductionApp() {
    const pageLayout = useRef(null);
    const searchText = useSelector(
        ({ machineProductionApp }) => machineProductionApp.machineProductions.searchText
    );

    return (
        <Root
            header={
                <GeneralHeader
                    iconType="settings_applications"
                    handleSearchTextChange={setMachineProductionSearchText}
                    pageLayout={pageLayout}
                    searchText={searchText}
                    headerName="Producción de Máquinas"
                />
            }
            content={<MachineProductionList />}
            leftSidebarContent={<MachineProductionSidebarContent />}
            sidebarInner
            ref={pageLayout}
            innerScroll
        />
    );
}

export default withReducer("machineProductionApp", reducer)(MachineProductionApp);
