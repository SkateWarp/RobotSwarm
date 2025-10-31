import { useEffect, useState, useMemo } from "react";
import PropTypes from "prop-types";
import { Box, Typography, Chip, IconButton, Icon, Paper, Button } from "@mui/material";
import { motion } from "framer-motion";
import axios from "axios";
import moment from "moment";
import { URL } from "../../../../constants/constants";
import jwtService from "../../../../services/jwtService";
import GeneralTableFrontendPaginationComplete from "app/shared-components/GeneralTableFrontendPaginationComplete";
import ChargingProgressBar from "app/shared-components/ChargingProgressBar";

function RobotTaskLogs({ robotId }) {
    const [taskLogs, setTaskLogs] = useState([]);
    const [loading, setLoading] = useState(true);

    const fetchTaskLogs = () => {
        setLoading(true);
        axios
            .get(`${URL}/TaskLog/${robotId}`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then((response) => {
                setTaskLogs(response.data);
                setLoading(false);
            })
            .catch((error) => {
                console.error("Error fetching task logs:", error);
                setLoading(false);
            });
    };

    const cancelAllTasks = () => {
        axios
            .put(`${URL}/TaskLog/cancel/robot/${robotId}`, {}, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then(() => {
                fetchTaskLogs(); // Refresh the list
            })
            .catch((error) => {
                console.error("Error cancelling tasks:", error);
            });
    };

    useEffect(() => {
        fetchTaskLogs();
    }, [robotId]);

    const getStatusColor = (taskLog) => {
        if (taskLog.dateCancelled) {
            return "error";
        }
        if (taskLog.dateFinished) {
            return "success";
        }
        return "warning";
    };

    const getStatusText = (taskLog) => {
        if (taskLog.dateCancelled) {
            return "Cancelado";
        }
        if (taskLog.dateFinished) {
            return "Completado";
        }
        return "Activo";
    };

    const columns = useMemo(
        () => [
            {
                Header: "Tarea",
                accessor: "taskTemplate.name",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        <Typography variant="body2" className="font-medium">
                            {row.original.taskTemplate?.name || "Tarea Desconocida"}
                        </Typography>
                    </div>
                ),
            },
            {
                Header: "Estado",
                accessor: "status",
                sortable: true,
                Cell: ({ row }) => (
                    <Chip
                        label={getStatusText(row.original)}
                        color={getStatusColor(row.original)}
                        size="small"
                    />
                ),
            },
            {
                Header: "Fecha Inicio",
                accessor: "dateCreated",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        {moment(row.original.dateCreated).format("DD-MM-YYYY HH:mm")}
                    </div>
                ),
            },
            {
                Header: "Fecha Fin",
                accessor: "dateFinished",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        {row.original.dateFinished
                            ? moment(row.original.dateFinished).format("DD-MM-YYYY HH:mm")
                            : row.original.dateCancelled
                            ? moment(row.original.dateCancelled).format("DD-MM-YYYY HH:mm")
                            : "-"}
                    </div>
                ),
            },
            {
                Header: "Duración",
                accessor: "duration",
                sortable: false,
                Cell: ({ row }) => {
                    const taskLog = row.original;
                    let duration = null;

                    if (taskLog.dateFinished) {
                        duration = moment(taskLog.dateFinished).diff(moment(taskLog.dateCreated));
                    } else if (taskLog.dateCancelled) {
                        duration = moment(taskLog.dateCancelled).diff(moment(taskLog.dateCreated));
                    } else {
                        duration = moment().diff(moment(taskLog.dateCreated));
                    }

                    if (duration) {
                        const hours = Math.floor(duration / 3600000);
                        const minutes = Math.floor((duration % 3600000) / 60000);
                        const seconds = Math.floor((duration % 60000) / 1000);

                        if (hours > 0) {
                            return `${hours}h ${minutes}m`;
                        }
                        return `${minutes}m ${seconds}s`;
                    }

                    return "-";
                },
            },
            {
                Header: "Parámetros",
                accessor: "parameters",
                sortable: false,
                Cell: ({ row }) => {
                    try {
                        const params = row.original.parameters ? JSON.stringify(row.original.parameters) : "-";
                        return (
                            <Typography variant="body2" className="font-mono text-xs max-w-200 truncate">
                                {params}
                            </Typography>
                        );
                    } catch (e) {
                        return "-";
                    }
                },
            },
        ],
        []
    );

    if (loading) {
        return <ChargingProgressBar />;
    }

    const activeTasks = taskLogs.filter(log => !log.dateFinished && !log.dateCancelled);

    return (
        <Box className="w-full">
            <Paper className="p-16 mb-16">
                <Box className="flex items-center justify-between">
                    <Box>
                        <Typography variant="h6" className="font-bold">
                            Resumen de Tareas
                        </Typography>
                        <Typography variant="body2" color="textSecondary">
                            Total: {taskLogs.length} tareas | Activas: {activeTasks.length}
                        </Typography>
                    </Box>
                    <Box className="flex gap-8">
                        <Button
                            variant="outlined"
                            color="primary"
                            onClick={fetchTaskLogs}
                            startIcon={<Icon>refresh</Icon>}
                        >
                            Actualizar
                        </Button>
                        {activeTasks.length > 0 && (
                            <Button
                                variant="contained"
                                color="error"
                                onClick={cancelAllTasks}
                                startIcon={<Icon>cancel</Icon>}
                            >
                                Cancelar Todas ({activeTasks.length})
                            </Button>
                        )}
                    </Box>
                </Box>
            </Paper>

            {taskLogs.length === 0 ? (
                <Box className="flex flex-1 items-center justify-center h-full p-64">
                    <Typography color="textSecondary" variant="h6">
                        No hay registros de tareas para este robot
                    </Typography>
                </Box>
            ) : (
                <motion.div
                    initial={{ y: 20, opacity: 0 }}
                    animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                    className="flex flex-auto w-full"
                >
                    <GeneralTableFrontendPaginationComplete
                        columns={columns}
                        data={taskLogs}
                        rowsPerPageOptions={[10, 25, 50]}
                    />
                </motion.div>
            )}
        </Box>
    );
}

RobotTaskLogs.propTypes = {
    robotId: PropTypes.string.isRequired,
};

export default RobotTaskLogs;
