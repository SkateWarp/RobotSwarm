import DialogContent from "@mui/material/DialogContent";
import { useCallback, useEffect } from "react";
import { useDispatch, useSelector } from "react-redux";
import { AppBar, Dialog, Icon, Toolbar, Typography, Grid, Card, CardContent, Chip, Box } from "@mui/material";
import moment from "moment";
import * as Actions from "app/store/fuse/messageSlice";
import GeneralDialogActionButtons from "app/shared-components/GeneralDialogActionButtons";
import { closeTaskLogDialog, selectTaskLogs } from "./store/taskLogSlice";

function TaskLogDialog() {
    const dispatch = useDispatch();
    const taskLogDialog = useSelector(
        ({ taskLogApp }) => taskLogApp.taskLogs.taskLogDialog
    );
    const taskLogs = useSelector(selectTaskLogs);

    const initDialog = useCallback(() => {
        // Dialog is already initialized in the slice
    }, []);

    useEffect(() => {
        if (taskLogDialog.props.open) {
            initDialog();
        }
    }, [taskLogDialog.props.open, initDialog]);

    function closeComposeDialog() {
        return dispatch(closeTaskLogDialog());
    }

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

    const getDuration = (taskLog) => {
        let duration = null;
        let endDate = null;

        if (taskLog.dateFinished) {
            duration = moment(taskLog.dateFinished).diff(moment(taskLog.dateCreated));
            endDate = taskLog.dateFinished;
        } else if (taskLog.dateCancelled) {
            duration = moment(taskLog.dateCancelled).diff(moment(taskLog.dateCreated));
            endDate = taskLog.dateCancelled;
        } else {
            duration = moment().diff(moment(taskLog.dateCreated));
            endDate = new Date();
        }

        if (duration) {
            const hours = Math.floor(duration / 3600000);
            const minutes = Math.floor((duration % 3600000) / 60000);
            const seconds = Math.floor((duration % 60000) / 1000);

            let durationText = "";
            if (hours > 0) durationText += `${hours}h `;
            if (minutes > 0) durationText += `${minutes}m `;
            if (seconds > 0 || durationText === "") durationText += `${seconds}s`;

            return {
                text: durationText.trim(),
                endDate: endDate
            };
        }

        return { text: "-", endDate: null };
    };

    if (!taskLogDialog.data) {
        return null;
    }

    const taskLog = taskLogDialog.data;
    const duration = getDuration(taskLog);

    return (
        <Dialog
            classes={{
                paper: "m-24",
            }}
            {...taskLogDialog.props}
            onClose={closeComposeDialog}
            fullWidth
            maxWidth="md"
        >
            <AppBar position="static" elevation={0}>
                <Toolbar className="flex w-full">
                    <Typography variant="subtitle1" color="inherit">
                        Task Log Details
                    </Typography>
                </Toolbar>
            </AppBar>

            <DialogContent classes={{ root: "p-24" }}>
                <Grid container spacing={3}>
                    {/* Basic Information */}
                    <Grid item xs={12}>
                        <Card>
                            <CardContent>
                                <Typography variant="h6" gutterBottom>
                                    Basic Information
                                </Typography>
                                <Grid container spacing={2}>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            Task Template
                                        </Typography>
                                        <Typography variant="body1" className="font-medium">
                                            {taskLog.taskTemplate?.name || "Unknown Task"}
                                        </Typography>
                                    </Grid>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            Status
                                        </Typography>
                                        <Chip
                                            label={getStatusText(taskLog)}
                                            color={getStatusColor(taskLog)}
                                            size="small"
                                        />
                                    </Grid>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            Start Date
                                        </Typography>
                                        <Typography variant="body1">
                                            {moment(taskLog.dateCreated).format("DD-MM-YYYY HH:mm:ss")}
                                        </Typography>
                                    </Grid>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            End Date
                                        </Typography>
                                        <Typography variant="body1">
                                            {taskLog.dateFinished
                                                ? moment(taskLog.dateFinished).format("DD-MM-YYYY HH:mm:ss")
                                                : taskLog.dateCancelled
                                                ? moment(taskLog.dateCancelled).format("DD-MM-YYYY HH:mm:ss")
                                                : "-"}
                                        </Typography>
                                    </Grid>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            Duration
                                        </Typography>
                                        <Typography variant="body1" className="font-medium">
                                            {duration.text}
                                        </Typography>
                                    </Grid>
                                    <Grid item xs={6}>
                                        <Typography variant="body2" color="textSecondary">
                                            Task Type
                                        </Typography>
                                        <Typography variant="body1">
                                            {taskLog.taskTemplate?.taskTypeDescription || "Unknown"}
                                        </Typography>
                                    </Grid>
                                </Grid>
                            </CardContent>
                        </Card>
                    </Grid>

                    {/* Robots Involved */}
                    <Grid item xs={12}>
                        <Card>
                            <CardContent>
                                <Typography variant="h6" gutterBottom>
                                    Robots Involved ({taskLog.robots?.length || 0})
                                </Typography>
                                <Box display="flex" flexWrap="wrap" gap={1}>
                                    {taskLog.robots?.map((robot) => (
                                        <Chip
                                            key={robot.id}
                                            label={`Robot ${robot.id}${robot.name ? ` - ${robot.name}` : ""}`}
                                            variant="outlined"
                                            color="primary"
                                            size="small"
                                        />
                                    )) || (
                                        <Typography variant="body2" color="textSecondary">
                                            No robots assigned to this task
                                        </Typography>
                                    )}
                                </Box>
                            </CardContent>
                        </Card>
                    </Grid>

                    {/* Parameters */}
                    <Grid item xs={12}>
                        <Card>
                            <CardContent>
                                <Typography variant="h6" gutterBottom>
                                    Parameters
                                </Typography>
                                {taskLog.parameters ? (
                                    <pre style={{
                                        backgroundColor: "#f5f5f5",
                                        padding: "16px",
                                        borderRadius: "4px",
                                        overflow: "auto",
                                        fontSize: "14px",
                                        whiteSpace: "pre-wrap"
                                    }}>
                                        {JSON.stringify(taskLog.parameters, null, 2)}
                                    </pre>
                                ) : (
                                    <Typography variant="body2" color="textSecondary">
                                        No parameters specified
                                    </Typography>
                                )}
                            </CardContent>
                        </Card>
                    </Grid>

                    {/* Task Template Details */}
                    {taskLog.taskTemplate && (
                        <Grid item xs={12}>
                            <Card>
                                <CardContent>
                                    <Typography variant="h6" gutterBottom>
                                        Task Template Details
                                    </Typography>
                                    <Grid container spacing={2}>
                                        <Grid item xs={6}>
                                            <Typography variant="body2" color="textSecondary">
                                                Template Name
                                            </Typography>
                                            <Typography variant="body1">
                                                {taskLog.taskTemplate.name}
                                            </Typography>
                                        </Grid>
                                        <Grid item xs={6}>
                                            <Typography variant="body2" color="textSecondary">
                                                Task Type
                                            </Typography>
                                            <Typography variant="body1">
                                                {taskLog.taskTemplate.taskTypeDescription}
                                            </Typography>
                                        </Grid>
                                        <Grid item xs={12}>
                                            <Typography variant="body2" color="textSecondary">
                                                Created
                                            </Typography>
                                            <Typography variant="body1">
                                                {moment(taskLog.taskTemplate.dateCreated).format("DD-MM-YYYY HH:mm:ss")}
                                            </Typography>
                                        </Grid>
                                    </Grid>
                                </CardContent>
                            </Card>
                        </Grid>
                    )}
                </Grid>
            </DialogContent>

            <GeneralDialogActionButtons dialogType="view" isValid={true} />
        </Dialog>
    );
}

export default TaskLogDialog;
