import { useEffect, useRef, useState } from "react";
import { useParams, useNavigate } from "react-router-dom";
import { Box, Typography, IconButton, Paper, Tab, Tabs } from "@mui/material";
import { ArrowBack } from "@mui/icons-material";
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import ChargingProgressBar from "app/shared-components/ChargingProgressBar";
import RobotSensorData from "./RobotSensorData";
import RobotTaskLogs from "./RobotTaskLogs";

function RobotDetailApp() {
    const { robotId } = useParams();
    const navigate = useNavigate();
    const [robot, setRobot] = useState(null);
    const [loading, setLoading] = useState(true);
    const [currentTab, setCurrentTab] = useState(0);

    useEffect(() => {
        // Fetch robot details
        axios
            .get(`${URL}/Robots/${robotId}`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then((response) => {
                setRobot(response.data);
                setLoading(false);
            })
            .catch((error) => {
                console.error("Error fetching robot details:", error);
                setLoading(false);
            });
    }, [robotId]);

    const handleTabChange = (event, newValue) => {
        setCurrentTab(newValue);
    };

    if (loading) {
        return <ChargingProgressBar />;
    }

    if (!robot) {
        return (
            <Box className="flex flex-1 items-center justify-center h-full">
                <Typography color="textSecondary" variant="h5">
                    Robot not found
                </Typography>
            </Box>
        );
    }

    return (
        <Box className="w-full h-full p-24">
            {/* Header */}
            <Box className="flex items-center gap-16 mb-24">
                <IconButton onClick={() => navigate("/apps/GTS/dashboard/booths")}>
                    <ArrowBack />
                </IconButton>
                <Box>
                    <Typography variant="h4" className="font-bold">
                        {robot.name}
                    </Typography>
                    <Typography variant="body1" color="textSecondary">
                        {robot.description}
                    </Typography>
                </Box>
            </Box>

            {/* Tabs */}
            <Paper className="mb-24">
                <Tabs
                    value={currentTab}
                    onChange={handleTabChange}
                    indicatorColor="primary"
                    textColor="primary"
                >
                    <Tab label="Sensor Data" />
                    <Tab label="Task Logs" />
                </Tabs>
            </Paper>

            {/* Tab Content */}
            <Box>
                {currentTab === 0 && <RobotSensorData robot={robot} />}
                {currentTab === 1 && <RobotTaskLogs robotId={robotId} />}
            </Box>
        </Box>
    );
}

export default RobotDetailApp;
