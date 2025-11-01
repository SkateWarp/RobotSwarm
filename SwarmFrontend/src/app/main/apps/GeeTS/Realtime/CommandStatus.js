import PropTypes from "prop-types";
import {
    Box,
    Card,
    CardContent,
    Chip,
    IconButton,
    Paper,
    Typography,
    List,
    ListItem,
    ListItemText,
    Divider,
    Collapse
} from "@mui/material";
import {
    Stop as StopIcon,
    ExpandMore as ExpandMoreIcon,
    ExpandLess as ExpandLessIcon,
    CheckCircle,
    PlayCircle,
    Error as ErrorIcon,
    History
} from "@mui/icons-material";
import { useState } from "react";

function CommandStatus({ runningCommands, commandHistory, onStopCommand }) {
    const [expandedHistory, setExpandedHistory] = useState(false);

    const formatDuration = (startTime) => {
        const now = new Date();
        const diff = now - new Date(startTime);
        const seconds = Math.floor(diff / 1000);
        const minutes = Math.floor(seconds / 60);
        const hours = Math.floor(minutes / 60);

        if (hours > 0) {
            return `${hours}h ${minutes % 60}m`;
        }
        if (minutes > 0) {
            return `${minutes}m ${seconds % 60}s`;
        }
        return `${seconds}s`;
    };

    const getStatusIcon = (status) => {
        switch (status) {
            case "running":
                return <PlayCircle color="success" />;
            case "sent":
                return <CheckCircle color="primary" />;
            case "stopped":
                return <StopIcon color="error" />;
            default:
                return <ErrorIcon color="warning" />;
        }
    };

    const runningCommandsList = Object.entries(runningCommands);

    return (
        <Box>
            {/* Running Commands */}
            {runningCommandsList.length > 0 && (
                <Card elevation={2} sx={{ mb: 3 }}>
                    <CardContent>
                        <Box className="flex items-center justify-between mb-16">
                            <Box className="flex items-center gap-8">
                                <PlayCircle color="success" />
                                <Typography variant="h6" className="font-bold">
                                    Active Commands
                                </Typography>
                                <Chip
                                    label={runningCommandsList.length}
                                    size="small"
                                    color="success"
                                />
                            </Box>
                        </Box>

                        <List>
                            {runningCommandsList.map(([robotId, commandInfo], index) => (
                                <Box key={robotId}>
                                    {index > 0 && <Divider />}
                                    <ListItem
                                        sx={{
                                            bgcolor: 'action.hover',
                                            borderRadius: 1,
                                            mb: 1
                                        }}
                                        secondaryAction={
                                            <IconButton
                                                edge="end"
                                                onClick={() => onStopCommand(parseInt(robotId))}
                                                color="error"
                                                sx={{
                                                    '&:hover': {
                                                        bgcolor: 'error.light',
                                                        color: 'error.contrastText'
                                                    }
                                                }}
                                            >
                                                <StopIcon />
                                            </IconButton>
                                        }
                                    >
                                        <ListItemText
                                            primary={
                                                <Box className="flex items-center gap-8">
                                                    <Typography variant="subtitle1" className="font-semibold">
                                                        Robot {robotId}
                                                    </Typography>
                                                    <Chip
                                                        label={commandInfo.command}
                                                        size="small"
                                                        color="success"
                                                        variant="outlined"
                                                    />
                                                </Box>
                                            }
                                            secondary={
                                                <Box className="flex items-center gap-16 mt-4">
                                                    <Typography variant="caption" color="textSecondary">
                                                        Started: {new Date(commandInfo.startTime).toLocaleTimeString()}
                                                    </Typography>
                                                    <Typography variant="caption" color="textSecondary">
                                                        Duration: {formatDuration(commandInfo.startTime)}
                                                    </Typography>
                                                    {commandInfo.taskLogId && (
                                                        <Typography variant="caption" color="textSecondary">
                                                            Task ID: {commandInfo.taskLogId}
                                                        </Typography>
                                                    )}
                                                </Box>
                                            }
                                        />
                                    </ListItem>
                                </Box>
                            ))}
                        </List>
                    </CardContent>
                </Card>
            )}

            {/* Command History */}
            {commandHistory.length > 0 && (
                <Card elevation={1}>
                    <CardContent>
                        <Box
                            className="flex items-center justify-between cursor-pointer"
                            onClick={() => setExpandedHistory(!expandedHistory)}
                        >
                            <Box className="flex items-center gap-8">
                                <History color="action" />
                                <Typography variant="h6" className="font-bold">
                                    Command History
                                </Typography>
                                <Chip
                                    label={commandHistory.length}
                                    size="small"
                                    color="default"
                                />
                            </Box>
                            {expandedHistory ? <ExpandLessIcon /> : <ExpandMoreIcon />}
                        </Box>

                        <Collapse in={expandedHistory}>
                            <List sx={{ mt: 2 }}>
                                {commandHistory.map((historyItem, index) => (
                                    <Box key={historyItem.id}>
                                        {index > 0 && <Divider />}
                                        <ListItem sx={{ px: 0 }}>
                                            <ListItemText
                                                primary={
                                                    <Box className="flex items-center gap-8">
                                                        {getStatusIcon(historyItem.status)}
                                                        <Typography variant="body2" className="font-medium">
                                                            Robot {historyItem.robotId}
                                                        </Typography>
                                                        <Chip
                                                            label={historyItem.command}
                                                            size="small"
                                                            color="primary"
                                                            variant="outlined"
                                                        />
                                                    </Box>
                                                }
                                                secondary={
                                                    <Typography variant="caption" color="textSecondary">
                                                        {historyItem.timestamp.toLocaleString()}
                                                    </Typography>
                                                }
                                            />
                                        </ListItem>
                                    </Box>
                                ))}
                            </List>
                        </Collapse>
                    </CardContent>
                </Card>
            )}

            {/* Empty State */}
            {runningCommandsList.length === 0 && commandHistory.length === 0 && (
                <Paper
                    variant="outlined"
                    sx={{
                        p: 4,
                        textAlign: 'center',
                        bgcolor: 'action.hover'
                    }}
                >
                    <History sx={{ fontSize: 48, color: 'text.secondary', mb: 2 }} />
                    <Typography variant="h6" color="textSecondary">
                        No commands sent yet
                    </Typography>
                    <Typography variant="body2" color="textSecondary">
                        Send a command to see it here
                    </Typography>
                </Paper>
            )}
        </Box>
    );
}

CommandStatus.propTypes = {
    runningCommands: PropTypes.object.isRequired,
    commandHistory: PropTypes.array.isRequired,
    onStopCommand: PropTypes.func.isRequired
};

export default CommandStatus;
