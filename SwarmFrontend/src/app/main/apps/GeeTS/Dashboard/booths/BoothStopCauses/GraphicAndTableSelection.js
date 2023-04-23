import { Tab, Tabs } from "@mui/material";
import { useState } from "react";
import PropTypes from "prop-types";
import NewStopCauseList from "./NewStopCauseList";
import NewStopCauseChart from "./NewStopCauseChart";

const GraphicAndTableSelection = ({ stopCauses }) => {
    const [selectedTab, setSelectedTab] = useState(0);

    const handleTabChange = (event, value) => {
        setSelectedTab(value);
    };

    return (
        <div className="widget w-full p-16 pb-32">
            <Tabs
                value={selectedTab}
                onChange={handleTabChange}
                indicatorColor="primary"
                textColor="primary"
                variant="scrollable"
                scrollButtons="auto"
                classes={{ root: "w-full h-64" }}
            >
                <Tab className="h-64 normal-case" label="Causas de paradas" />
                <Tab className="h-64 normal-case" label="Gráfica de paradas" />
            </Tabs>

            {selectedTab === 0 && <NewStopCauseList stopCauses={stopCauses} />}

            {selectedTab === 1 && <NewStopCauseChart />}
        </div>
    );
};

GraphicAndTableSelection.propTypes = {
    stopCauses: PropTypes.array.isRequired,
};

export default GraphicAndTableSelection;
