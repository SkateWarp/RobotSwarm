import withReducer from "app/store/withReducer";
import SimulationWorkspace from "./SimulationWorkspace";
import reducer from "./store";

function RealtimeConfigApp() {
    return <SimulationWorkspace />;
}

export default withReducer("realtimeConfigApp", reducer)(RealtimeConfigApp);
