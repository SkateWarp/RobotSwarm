import { useEffect, useState, useRef } from "react";
import PropTypes from "prop-types";
import {
    Box,
    Chip,
    Grid,
    Paper,
    Tooltip,
    Typography
} from "@mui/material";
import {
    Pause,
    PlayArrow,
    Timeline
} from '@mui/icons-material';
import axios from "axios";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import singletonInstance from "../../../../services/SignalRService/signalRConnectionService";
import moment from 'moment';
import SensorChart from './SensorChart';

function RobotSensorData({ robot }) {
    const connectionRef = useRef(null);
    const eventHandlerRef = useRef(null);
    const [readings, setReadings] = useState([]);
    const [isConnected, setIsConnected] = useState(robot.isConnected);
    const [currentStatus, setCurrentStatus] = useState(robot.status);
    const [sensorHistory, setSensorHistory] = useState({});

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
                return <Pause sx={{ fontSize: 20 }} />;
            case 1:
                return <PlayArrow sx={{ fontSize: 20 }} />;
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
            default:
                return "Desconocido";
        }
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

                // Initialize history for each sensor
                const initialHistory = {};
                sortedReadings.forEach(reading => {
                    initialHistory[reading.notes] = [{
                        time: moment().format('HH:mm:ss'),
                        value: parseFloat(reading.value)
                    }];
                });
                setSensorHistory(initialHistory);
            })
            .catch((error) => {
                console.error("Error fetching sensor readings:", error);
            });

        // Set up SignalR event handlers
        eventHandlerRef.current = (current) => {
            const sortedReadings = current.sort((a, b) => a.notes.localeCompare(b.notes));
            setReadings(sortedReadings);

            // Update history (keep last 20 readings per sensor)
            setSensorHistory(prevHistory => {
                const newHistory = { ...prevHistory };
                sortedReadings.forEach(reading => {
                    const sensorName = reading.notes;
                    const newDataPoint = {
                        time: moment().format('HH:mm:ss'),
                        value: parseFloat(reading.value)
                    };

                    if (!newHistory[sensorName]) {
                        newHistory[sensorName] = [];
                    }

                    newHistory[sensorName] = [...newHistory[sensorName], newDataPoint].slice(-20);
                });
                return newHistory;
            });
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
        <Box className="w-full">
            {/* Status Section */}
            <Paper className="p-24 mb-24">
                <Grid container spacing={3} alignItems="center">
                    <Grid item xs={12} md={4}>
                        <Box className="flex items-center gap-12">
                            <Timeline color="primary" sx={{ fontSize: 40 }} />
                            <Box>
                                <Typography variant="body2" color="textSecondary">
                                    Estado
                                </Typography>
                                <Chip
                                    label={statusDescription(currentStatus)}
                                    icon={<StatusIcon />}
                                    color={currentStatus === 1 ? "success" : "default"}
                                    sx={{ mt: 1 }}
                                />
                            </Box>
                        </Box>
                    </Grid>
                    <Grid item xs={12} md={4}>
                        <Box className="flex items-center gap-12">
                            <Box>
                                <Typography variant="body2" color="textSecondary">
                                    Conexión
                                </Typography>
                                <Box className="flex items-center gap-8 mt-8">
                                    <Box
                                        className="w-12 h-12 rounded-full"
                                        sx={{
                                            backgroundColor: isConnected ? "success.main" : "error.main",
                                        }}
                                    />
                                    <Typography variant="body1">
                                        {isConnected ? "Conectado" : "Desconectado"}
                                    </Typography>
                                </Box>
                            </Box>
                        </Box>
                    </Grid>
                </Grid>
            </Paper>

            {/* Current Sensor Readings */}
            <Typography variant="h5" className="font-bold mb-16">
                Lecturas Actuales
            </Typography>
            <Grid container spacing={2} className="mb-32">
                {/* Left Wheel Section */}
                {sensorGroups.left.length > 0 && (
                    <Grid item xs={12} md={6}>
                        <Paper elevation={2} className="p-16">
                            <Typography
                                variant="h6"
                                className="font-semibold mb-16 text-center"
                                color="primary"
                                sx={{ fontSize: '1.25rem' }}
                            >
                                IZQUIERDA
                            </Typography>
                            <Box className="space-y-12">
                                {sensorGroups.left.map((sensor) => (
                                    <Box key={sensor.id} className="flex items-center justify-between gap-12 p-12" sx={{ bgcolor: 'action.hover', borderRadius: 1 }}>
                                        <Typography
                                            variant="body1"
                                            color="text.primary"
                                            sx={{
                                                fontSize: '1.125rem',
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
                                            sx={{ fontSize: '1.5rem', flexShrink: 0, textAlign: 'right' }}
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
                    <Grid item xs={12} md={6}>
                        <Paper elevation={2} className="p-16">
                            <Typography
                                variant="h6"
                                className="font-semibold mb-16 text-center"
                                color="primary"
                                sx={{ fontSize: '1.25rem' }}
                            >
                                DERECHA
                            </Typography>
                            <Box className="space-y-12">
                                {sensorGroups.right.map((sensor) => (
                                    <Box key={sensor.id} className="flex items-center justify-between gap-12 p-12" sx={{ bgcolor: 'action.hover', borderRadius: 1 }}>
                                        <Typography
                                            variant="body1"
                                            color="text.primary"
                                            sx={{
                                                fontSize: '1.125rem',
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
                                            sx={{ fontSize: '1.5rem', flexShrink: 0, textAlign: 'right' }}
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
                        <Paper elevation={2} className="p-16">
                            <Typography
                                variant="h6"
                                className="font-semibold mb-16 text-center"
                                color="primary"
                                sx={{ fontSize: '1.25rem' }}
                            >
                                SISTEMA
                            </Typography>
                            <Grid container spacing={2}>
                                {sensorGroups.other.map((sensor) => (
                                    <Grid item xs={12} sm={6} md={4} key={sensor.id}>
                                        <Box className="flex items-center justify-between gap-12 p-12" sx={{ bgcolor: 'action.hover', borderRadius: 1 }}>
                                            <Typography
                                                variant="body1"
                                                color="text.primary"
                                                sx={{
                                                    fontSize: '1.125rem',
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
                                                sx={{ fontSize: '1.5rem', flexShrink: 0, textAlign: 'right' }}
                                            >
                                                {parseFloat(sensor.value).toFixed(1)}
                                            </Typography>
                                        </Box>
                                    </Grid>
                                ))}
                            </Grid>
                        </Paper>
                    </Grid>
                )}
            </Grid>

            {/* Real-time Charts */}
            <Typography variant="h5" className="font-bold mb-16">
                Gráficos en Tiempo Real
            </Typography>
            <Grid container spacing={2}>
                {Object.entries(sensorHistory).map(([sensorName, data]) => (
                    <Grid item xs={12} md={6} key={sensorName}>
                        <SensorChart sensorName={sensorName} data={data} />
                    </Grid>
                ))}
            </Grid>
        </Box>
    );
}

RobotSensorData.propTypes = {
    robot: PropTypes.object.isRequired,
};

export default RobotSensorData;
