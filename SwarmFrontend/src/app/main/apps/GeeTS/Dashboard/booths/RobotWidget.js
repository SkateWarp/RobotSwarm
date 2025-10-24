import { memo, useEffect, useState, useRef } from "react";
import PropTypes from "prop-types";
import { Table, TableBody, TableCell, TableRow, Tooltip, Typography } from "@mui/material";
import { Pause, PauseCircleOutline, PlayArrow } from '@mui/icons-material';
import axios from "axios";
import { LOGO, URL } from "../../../../../constants/constants";
import jwtService from "../../../../../services/jwtService";
import singletonInstance from "../../../../../services/SignalRService/signalRConnectionService";

function RobotWidget({ robot }) {
    const connectionRef = useRef(null);
    const eventHandlerRef = useRef(null);
    const [readings, setReadings] = useState([]);
    const [isConnected, setIsConnected] = useState(robot.isConnected);
    const [currentStatus, setCurrentStatus] = useState(robot.status);

    const StatusIcon = () => {
        switch (currentStatus) {
            case 0:
                return <Pause sx={{ fontSize: 48, color: 'gray' }} />;
            case 1:
                return <PlayArrow sx={{ fontSize: 48, color: 'green' }} />;
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
            console.log(`Received AllSensorReadings for robot ${robot.id}:`, current);
            const sortedReadings = current.sort((a, b) => a.notes.localeCompare(b.notes));
            setReadings(sortedReadings);
        };

        // Wait for connection to be established before registering handlers
        singletonInstance.getConnectionPromise(`robot_${robot.id}`)
            .then((connection) => {
                console.log(`Registering event handlers for robot ${robot.id}`);

                // Register the event handlers
                connection.on(`AllSensorReadings/${robot.id}`, eventHandlerRef.current);
                connection.on(`RobotConnectionChanged/${robot.id}`, (params) => {
                    console.log(`Robot ${robot.id} connection changed:`, params);
                    setIsConnected(params.isConnected);
                });
                connection.on(`RobotStatusChanged/${robot.id}`, (params) => {
                    console.log(`Robot ${robot.id} status changed:`, params);
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
        <div className="w-full p-32">
            <div className="flex w-full justify-between">
                <div>
                    <Tooltip title={isConnected ? "Conectado" : "Desconectado"} arrow>
                        <div className="w-36 h-36 flex justify-center items-center">
                            <div
                                className="h2 p-8"
                                style={{
                                    backgroundColor: isConnected ? "green" : "red",
                                    display: "inline-flex",
                                    borderRadius: "25px",
                                }}
                            />
                        </div>
                    </Tooltip>
                </div>
                <div>
                    <Tooltip title={statusDescription(currentStatus)} arrow>
                        <div className="w-36 h-36 flex justify-center items-center">
                            <StatusIcon />
                        </div>
                    </Tooltip>
                </div>
            </div>
            <div className="flex flex-row flex-wrap items-end">
                <div className="flex flex-col m-auto">
                    <img alt="logo" src={LOGO} className="inline-flex w-192 leading-none mx-auto" />
                </div>
            </div>
            <div className="flex flex-row flex-wrap items-end">
                <div className="flex flex-col m-auto">
                    <Typography
                        className="inline-flex px-16 items-center py-auto text-32"
                        color="textSecondary"
                    >
                        {robot?.name}
                    </Typography>
                </div>
            </div>
            <Table className="mt-8 text-center bg-transparent">
                <TableBody>
                    <TableRow
                        style={{
                            height: "260px",
                        }}
                    >
                        <TableCell>
                            <div className="flex flex-col">
                                <div className="flex flex-col items-center">
                                    <Typography className="flex h2 text-center justify-items-center items-center self-center mt-16">
                                        {robot?.description}
                                    </Typography>
                                    <Typography className="h3" color="textSecondary">
                                        Descripcion
                                    </Typography>
                                </div>

                                <div className="flex flex-col items-center">
                                    <Typography className="flex h2 text-center justify-items-center items-center self-center mt-16">
                                        {robot?.statusDescription}
                                    </Typography>
                                    <Typography className="h3" color="textSecondary">
                                        Status
                                    </Typography>
                                </div>

                                <div className="overflow-y-auto max-h-80 p-4">
                                    {readings.map((reading, index) => (
                                        <div key={reading.id} className="flex flex-col items-center mb-4">
                                            <Typography className="flex h2 text-center justify-items-center items-center self-center mt-16">
                                                {reading.notes} : {reading.value}
                                            </Typography>
                                            <Typography className="h3" color="textSecondary">
                                                Sensor #{index + 1}
                                            </Typography>
                                        </div>
                                    ))}
                                </div>
                            </div>
                        </TableCell>
                    </TableRow>
                </TableBody>
            </Table>
        </div>
    );
}

RobotWidget.propTypes = {
    robot: PropTypes.object.isRequired,
};

export default memo(RobotWidget);
