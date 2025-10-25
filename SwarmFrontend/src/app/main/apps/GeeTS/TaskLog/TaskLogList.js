import { motion } from "framer-motion";
import { useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { Icon, IconButton, Typography, Chip } from "@mui/material";
import moment from "moment";
import ChargingProgressBar from "app/shared-components/ChargingProgressBar";
import useFilteredData from "app/shared-components/hooks/useFilteredData";
import GeneralTableFrontendPaginationComplete from "app/shared-components/GeneralTableFrontendPaginationComplete";
import { openTaskLogDialog, selectTaskLogs } from "./store/taskLogSlice";

function TaskLogList() {
    const dispatch = useDispatch();
    const taskLogs = useSelector(selectTaskLogs);

    const { statusFilter, filteredData } = useFilteredData(taskLogs, "");
    const [openDialog, setOpenDialog] = useState(false);

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
            return "Cancelled";
        }
        if (taskLog.dateFinished) {
            return "Completed";
        }
        return "Active";
    };

    const columns = useMemo(
        () => [
            {
                Header: "Task Template",
                accessor: "taskTemplate.name",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        <Typography variant="body2" className="font-medium">
                            {row.original.taskTemplate?.name || "Unknown Task"}
                        </Typography>
                    </div>
                ),
            },
            {
                Header: "Robots",
                accessor: "robots",
                sortable: false,
                Cell: ({ row }) => (
                    <div className="flex flex-wrap gap-1">
                        {row.original.robots?.map((robot) => (
                            <Chip
                                key={robot.id}
                                label={`Robot ${robot.id}`}
                                size="small"
                                variant="outlined"
                                color="primary"
                            />
                        )) || <Typography variant="body2" color="textSecondary">No robots</Typography>}
                    </div>
                ),
            },
            {
                Header: "Status",
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
                Header: "Start Date",
                accessor: "dateCreated",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        {moment(row.original.dateCreated).format("DD-MM-YYYY HH:mm")}
                    </div>
                ),
            },
            {
                Header: "End Date",
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
                Header: "Duration",
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
                        const minutes = Math.floor(duration / 60000);
                        const seconds = Math.floor((duration % 60000) / 1000);
                        return `${minutes}m ${seconds}s`;
                    }

                    return "-";
                },
            },
            {
                id: "action",
                Header: "View",
                className: "justify-center",
                width: 128,
                sortable: false,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        <IconButton
                            onClick={(ev) => {
                                ev.stopPropagation();
                                dispatch(openTaskLogDialog(row.original));
                            }}
                        >
                            <Icon>visibility</Icon>
                        </IconButton>
                    </div>
                ),
            },
        ],
        [dispatch]
    );

    if (statusFilter && !filteredData.length) {
        return (
            <div className="flex flex-1 items-center justify-center h-full">
                <Typography color="textSecondary" variant="h5">
                    No task logs found!
                </Typography>
            </div>
        );
    }

    if (!statusFilter) {
        return <ChargingProgressBar />;
    }

    return (
        <motion.div
            initial={{ y: 20, opacity: 0 }}
            animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
            className="flex flex-auto w-full max-h-full"
        >
            <GeneralTableFrontendPaginationComplete
                columns={columns}
                data={filteredData}
                onRowClick={(ev, row) => {
                    if (row) {
                        dispatch(openTaskLogDialog(row.original));
                    }
                }}
                rowsPerPageOptions={[5, 10, 15, 25]}
            />
        </motion.div>
    );
}

export default TaskLogList;
