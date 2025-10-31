import { memo, useEffect, useState, useRef } from "react";
import PropTypes from "prop-types";
import { useNavigate } from "react-router-dom";
import {
    Box,
    Chip,
    Divider,
    Grid,
    Paper,
    Tooltip,
    Typography
} from "@mui/material";
import {
    Pause,
    PlayArrow
} from '@mui/icons-material';
import axios from "axios";
import { LOGO, URL } from "../../../../../constants/constants";
import jwtService from "../../../../../services/jwtService";
import singletonInstance from "../../../../../services/SignalRService/signalRConnectionService";

function RobotWidget({ robot }) {
    const navigate = useNavigate();
    const connectionRef = useRef(null);
    const eventHandlerRef = useRef(null);
    const [readings, setReadings] = useState([]);
    const [isConnected, setIsConnected] = useState(robot.isConnected);
    const [currentStatus, setCurrentStatus] = useState(robot.status);

    // Dynamically group sensors by prefix (left_, right_, or other)
    const groupSensors = () => {
        const groups = {
            left: [],
            right: [],
            other: []
        };

        readings.forEach(reading => {
            if (reading.notes.startsWith('left_')) {
                groups.left.push(reading);
            } else if (reading.notes.startsWith('right_')) {
                groups.right.push(reading);
            } else {
                groups.other.push(reading);
            }
        });

        return groups;
    };

    const sensorGroups = groupSensors();

    const StatusIcon = () => {
        switch (currentStatus) {
            case 0:
                return <Pause sx={{ fontSize: 16, color: 'gray' }} />;
            case 1:
                return <PlayArrow sx={{ fontSize: 16, color: 'green' }} />;
            default:
                return null;
        }
    };

    const statusDescription = (status) => {
        switch (status) {
            case 0:
                return "Reposo";
            case 1:
                return "Trabajando";

        };
        return "Desconocido";
    };

    useEffect(() => {
        // Get or create a connection for this specific robot
        connectionRef.current = singletonInstance.createConnectionBuilder(`robot_${robot.id}`);

        // Fetch initial sensor readings
        axios
            .get(`${URL}/SensorReadings/last/${robot.id}`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then((response) => {
                const sortedReadings = response.data.sort((a, b) => a.notes.localeCompare(b.notes));
                setReadings(sortedReadings);
            })
            .catch((error) => {
                console.error("Error fetching sensor readings:", error);
            });

        // Set up SignalR event handlers
        eventHandlerRef.current = (current) => {
            const sortedReadings = current.sort((a, b) => a.notes.localeCompare(b.notes));
            setReadings(sortedReadings);
        };

        // Wait for connection to be established before registering handlers
        singletonInstance.getConnectionPromise(`robot_${robot.id}`)
            .then((connection) => {

                // Register the event handlers
                connection.on(`AllSensorReadings/${robot.id}`, eventHandlerRef.current);
                connection.on(`RobotConnectionChanged/${robot.id}`, (params) => {
                    setIsConnected(params.isConnected);
                });
                connection.on(`RobotStatusChanged/${robot.id}`, (params) => {
                    setCurrentStatus(params.status);
                });
            })
            .catch((error) => {
                console.error(`Error establishing SignalR connection for robot ${robot.id}:`, error);
            });

        // Cleanup function to remove event handlers when component unmounts
        return () => {
            if (connectionRef.current && eventHandlerRef.current) {
                connectionRef.current.off(`AllSensorReadings/${robot.id}`, eventHandlerRef.current);
                connectionRef.current.off(`RobotConnectionChanged/${robot.id}`);
                connectionRef.current.off(`RobotStatusChanged/${robot.id}`);
            }
        };
    }, [robot.id]);

    return (
        <Box
            className="w-full p-12 cursor-pointer transition-all hover:bg-gray-50"
            onClick={() => navigate(`/apps/GTS/robot/${robot.id}`)}
            sx={{
                '&:hover': {
                    transform: 'scale(1.02)',
                    boxShadow: 3,
                }
            }}
        >
            {/* Header Section */}
            <Box className="flex items-center justify-between mb-12">
                <Box className="flex items-center gap-6">
                    <img alt="logo" src={LOGO} className="w-32 h-32" />
                    <Box>
                        <Typography variant="h6" className="font-semibold leading-tight" sx={{ fontSize: '1.125rem' }}>
                            {robot?.name}
                        </Typography>
                        <Typography variant="body2" color="textSecondary" sx={{ fontSize: '0.875rem' }}>
                            {robot?.description}
                        </Typography>
                    </Box>
                </Box>

                <Box className="flex flex-col items-end gap-4">
                    <Chip
                        label={statusDescription(currentStatus)}
                        icon={<StatusIcon />}
                        color={currentStatus === 1 ? "success" : "default"}
                        size="small"
                        sx={{ height: 24, fontSize: '0.875rem', '& .MuiChip-icon': { fontSize: 16 } }}
                    />
                    <Tooltip title={isConnected ? "Conectado" : "Desconectado"} arrow>
                        <Box
                            className="w-10 h-10 rounded-full"
                            sx={{
                                backgroundColor: isConnected ? "success.main" : "error.main",
                            }}
                        />
                    </Tooltip>
                </Box>
            </Box>

            <Divider className="mb-12" />

            {/* Sensor Data Grid */}
            <Grid container spacing={2}>
                {/* Left Wheel Section */}
                {sensorGroups.left.length > 0 && (
                    <Grid item xs={6}>
                        <Paper elevation={0} className="p-12" sx={{ bgcolor: 'action.hover' }}>
                            <Typography
                                variant="h6"
                                className="font-semibold mb-12 block text-center"
                                color="primary"
                                sx={{ fontSize: '1.125rem' }}
                            >
                                IZQUIERDA
                            </Typography>
                            <Box className="space-y-8">
                                {sensorGroups.left.map((sensor) => (
                                    <Box key={sensor.id} className="flex items-center justify-between gap-12">
                                        <Typography
                                            variant="body1"
                                            color="text.primary"
                                            sx={{
                                                fontSize: '1rem',
                                                textTransform: 'uppercase',
                                                fontWeight: 500,
                                                flex: 1,
                                                textAlign: 'left'
                                            }}
                                        >
                                            {sensor.notes.replace('left_', '').replace(/_/g, ' ')}
                                        </Typography>
                                        <Typography
                                            variant="h5"
                                            className="font-mono font-bold"
                                            sx={{ fontSize: '1.25rem', flexShrink: 0, textAlign: 'right' }}
                                        >
                                            {parseFloat(sensor.value).toFixed(1)}
                                        </Typography>
                                    </Box>
                                ))}
                            </Box>
                        </Paper>
                    </Grid>
                )}

                {/* Right Wheel Section */}
                {sensorGroups.right.length > 0 && (
                    <Grid item xs={6}>
                        <Paper elevation={0} className="p-12" sx={{ bgcolor: 'action.hover' }}>
                            <Typography
                                variant="h6"
                                className="font-semibold mb-12 block text-center"
                                color="primary"
                                sx={{ fontSize: '1.125rem' }}
                            >
                                DERECHA
                            </Typography>
                            <Box className="space-y-8">
                                {sensorGroups.right.map((sensor) => (
                                    <Box key={sensor.id} className="flex items-center justify-between gap-12">
                                        <Typography
                                            variant="body1"
                                            color="text.primary"
                                            sx={{
                                                fontSize: '1rem',
                                                textTransform: 'uppercase',
                                                fontWeight: 500,
                                                flex: 1,
                                                textAlign: 'left'
                                            }}
                                        >
                                            {sensor.notes.replace('right_', '').replace(/_/g, ' ')}
                                        </Typography>
                                        <Typography
                                            variant="h5"
                                            className="font-mono font-bold"
                                            sx={{ fontSize: '1.25rem', flexShrink: 0, textAlign: 'right' }}
                                        >
                                            {parseFloat(sensor.value).toFixed(1)}
                                        </Typography>
                                    </Box>
                                ))}
                            </Box>
                        </Paper>
                    </Grid>
                )}

                {/* Other Sensors Section */}
                {sensorGroups.other.length > 0 && (
                    <Grid item xs={12}>
                        <Paper elevation={0} className="p-12 mt-8" sx={{ bgcolor: 'action.hover' }}>
                            <Typography
                                variant="h6"
                                className="font-semibold mb-12 block text-center"
                                color="primary"
                                sx={{ fontSize: '1.125rem' }}
                            >
                                SISTEMA
                            </Typography>
                            <Box className="space-y-8">
                                {sensorGroups.other.map((sensor) => (
                                    <Box key={sensor.id} className="flex items-center justify-between gap-12">
                                        <Typography
                                            variant="body1"
                                            color="text.primary"
                                            sx={{
                                                fontSize: '1rem',
                                                textTransform: 'uppercase',
                                                fontWeight: 500,
                                                flex: 1,
                                                textAlign: 'left'
                                            }}
                                        >
                                            {sensor.notes.replace(/_/g, ' ')}
                                        </Typography>
                                        <Typography
                                            variant="h5"
                                            className="font-mono font-bold"
                                            sx={{ fontSize: '1.25rem', flexShrink: 0, textAlign: 'right' }}
                                        >
                                            {parseFloat(sensor.value).toFixed(1)}
                                        </Typography>
                                    </Box>
                                ))}
                            </Box>
                        </Paper>
                    </Grid>
                )}
            </Grid>
        </Box>
    );
}

RobotWidget.propTypes = {
    robot: PropTypes.object.isRequired,
};

export default memo(RobotWidget);
