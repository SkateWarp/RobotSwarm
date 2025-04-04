import { memo, useEffect, useState, useRef } from "react";
import PropTypes from "prop-types";
import { Table, TableBody, TableCell, TableRow, Tooltip, Typography } from "@mui/material";
import axios from "axios";
import { LOGO, URL } from "../../../../../constants/constants";
import jwtService from "../../../../../services/jwtService";
import singletonInstance from "../../../../../services/SignalRService/signalRConnectionService";

function RobotWidget({ robot }) {
    const [readings, setReadings] = useState([]);
    const connectionRef = useRef(null);
    const eventHandlerRef = useRef(null);

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
                setReadings(response.data);
            })
            .catch((error) => {
                console.error("Error fetching sensor readings:", error);
            });

        // Set up SignalR event handlers
        eventHandlerRef.current = (current) => {
            console.debug(`Received sensor readings for robot ${robot.id}:`, current);
            setReadings(current);
        };

        // Register the event handler
        connectionRef.current.on(`AllSensorReadings/${robot.id}`, eventHandlerRef.current);

        // Cleanup function to remove event handlers when component unmounts
        return () => {
            if (connectionRef.current && eventHandlerRef.current) {
                connectionRef.current.off(`AllSensorReadings/${robot.id}`, eventHandlerRef.current);
            }
        };
    }, [robot.id]);

    return (
        <div className="w-full p-32">
            <div className="flex w-full justify-between">
                <div>
                    <Tooltip title="Encendida" arrow>
                        <div className="w-36 h-36">
                            <div
                                className="h2 p-8"
                                style={{
                                    backgroundColor: robot.isConnected ? "green" : "red",
                                    display: "inline-flex",
                                    borderRadius: "25px",
                                    left: "80%",
                                }}
                            />
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
                                        <div key={index} className="flex flex-col items-center mb-4">
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
