import { useState } from "react";
import PropTypes from "prop-types";
import {
    Box,
    Button,
    Card,
    CardContent,
    Chip,
    FormControl,
    Grid,
    InputLabel,
    MenuItem,
    Paper,
    Select,
    TextField,
    Typography,
    Accordion,
    AccordionSummary,
    AccordionDetails,
    IconButton,
    Tooltip
} from "@mui/material";
import {
    Send as SendIcon,
    ExpandMore as ExpandMoreIcon,
    PlayArrow,
    Stop,
    Sensors,
    Settings,
    Code
} from "@mui/icons-material";

const COMMAND_TEMPLATES = {
    sensor_data: {
        name: "Sensor Data",
        icon: <Sensors />,
        color: "primary",
        template: {
            left_ticks: 0,
            right_ticks: 0,
            left_diff: 0,
            right_diff: 0,
            left_dist: 0,
            right_dist: 0,
            timestep: 0,
            left_speed: 0,
            right_speed: 0,
            left_speed_filtered: 0,
            right_speed_filtered: 0
        }
    },
    task: {
        name: "Task",
        icon: <PlayArrow />,
        color: "success",
        template: {
            taskType: "transporte",
            parameters: {
                sensorName: "speed",
                value: 0
            }
        }
    },
    status: {
        name: "Status",
        icon: <Settings />,
        color: "warning",
        options: ["Working", "Idle", "Error", "Maintenance"]
    }
};

function CommandPanel({
    robots,
    selectedRobot,
    onRobotChange,
    onSendCommand,
    connectionStatus,
    userId
}) {
    const [commandType, setCommandType] = useState("");
    const [commandData, setCommandData] = useState("");
    const [rawMode, setRawMode] = useState(false);

    const handleCommandTypeChange = (type) => {
        setCommandType(type);
        const template = COMMAND_TEMPLATES[type];

        if (template.template) {
            setCommandData(JSON.stringify(template.template, null, 2));
        } else if (template.options) {
            setCommandData(template.options[0]);
        }
    };

    const handleQuickSend = (type) => {
        handleCommandTypeChange(type);
        setTimeout(() => {
            handleSend();
        }, 100);
    };

    const handleSend = () => {
        if (!selectedRobot || !commandType || !commandData) return;

        let parsedData = commandData;

        // Parse JSON if needed
        if (commandType !== "status") {
            try {
                parsedData = JSON.parse(commandData);
            } catch (error) {
                alert("Invalid JSON format");
                return;
            }
        }

        onSendCommand(selectedRobot, commandType, parsedData);
    };

    const handleFieldChange = (path, value) => {
        try {
            const data = JSON.parse(commandData);
            const keys = path.split('.');
            let current = data;

            for (let i = 0; i < keys.length - 1; i++) {
                current = current[keys[i]];
            }

            current[keys[keys.length - 1]] = parseFloat(value) || value;
            setCommandData(JSON.stringify(data, null, 2));
        } catch (error) {
            console.error("Error updating field:", error);
        }
    };

    const renderFormFields = () => {
        if (rawMode || !commandType) return null;

        const template = COMMAND_TEMPLATES[commandType];

        // Status - dropdown
        if (commandType === "status") {
            return (
                <FormControl fullWidth sx={{ mt: 2 }}>
                    <InputLabel>Status</InputLabel>
                    <Select
                        value={commandData}
                        onChange={(e) => setCommandData(e.target.value)}
                        label="Status"
                    >
                        {template.options.map((option) => (
                            <MenuItem key={option} value={option}>
                                {option}
                            </MenuItem>
                        ))}
                    </Select>
                </FormControl>
            );
        }

        // Sensor Data - form fields
        if (commandType === "sensor_data") {
            const data = JSON.parse(commandData);
            return (
                <Grid container spacing={2} sx={{ mt: 1 }}>
                    {Object.keys(template.template).map((key) => (
                        <Grid item xs={6} md={4} key={key}>
                            <TextField
                                fullWidth
                                size="small"
                                label={key.replace(/_/g, ' ').toUpperCase()}
                                type="number"
                                value={data[key] || 0}
                                onChange={(e) => handleFieldChange(key, e.target.value)}
                            />
                        </Grid>
                    ))}
                </Grid>
            );
        }

        // Task - nested form fields
        if (commandType === "task") {
            const data = JSON.parse(commandData);
            return (
                <Grid container spacing={2} sx={{ mt: 1 }}>
                    <Grid item xs={12}>
                        <TextField
                            fullWidth
                            size="small"
                            label="Task Type"
                            value={data.taskType || ""}
                            onChange={(e) => handleFieldChange("taskType", e.target.value)}
                        />
                    </Grid>
                    <Grid item xs={6}>
                        <TextField
                            fullWidth
                            size="small"
                            label="Sensor Name"
                            value={data.parameters?.sensorName || ""}
                            onChange={(e) => handleFieldChange("parameters.sensorName", e.target.value)}
                        />
                    </Grid>
                    <Grid item xs={6}>
                        <TextField
                            fullWidth
                            size="small"
                            label="Value"
                            type="number"
                            value={data.parameters?.value || 0}
                            onChange={(e) => handleFieldChange("parameters.value", e.target.value)}
                        />
                    </Grid>
                </Grid>
            );
        }

        return null;
    };

    return (
        <Card elevation={3}>
            <CardContent>
                <Typography variant="h6" className="font-bold mb-16">
                    Send Command
                </Typography>

                {/* Robot Selection */}
                <FormControl fullWidth sx={{ mb: 3 }}>
                    <InputLabel>Select Robot</InputLabel>
                    <Select
                        value={selectedRobot}
                        onChange={(e) => onRobotChange(e.target.value)}
                        label="Select Robot"
                    >
                        {robots.map((robot) => (
                            <MenuItem key={robot.id} value={robot.id}>
                                <Box className="flex items-center justify-between w-full">
                                    <span>Robot {robot.id} - {robot.name}</span>
                                    <Chip
                                        label={robot.accountId === userId ? "Mine" : robot.isPublic ? "Public" : "Private"}
                                        size="small"
                                        color={robot.accountId === userId ? "primary" : "default"}
                                        sx={{ ml: 2 }}
                                    />
                                </Box>
                            </MenuItem>
                        ))}
                    </Select>
                </FormControl>

                {/* Quick Action Buttons */}
                <Box sx={{ mb: 3 }}>
                    <Typography variant="subtitle2" className="mb-8" color="textSecondary">
                        Quick Actions
                    </Typography>
                    <Grid container spacing={2}>
                        {Object.entries(COMMAND_TEMPLATES).map(([key, template]) => (
                            <Grid item xs={12} sm={4} key={key}>
                                <Button
                                    fullWidth
                                    variant={commandType === key ? "contained" : "outlined"}
                                    color={template.color}
                                    startIcon={template.icon}
                                    onClick={() => handleCommandTypeChange(key)}
                                    disabled={!selectedRobot || connectionStatus !== "connected"}
                                >
                                    {template.name}
                                </Button>
                            </Grid>
                        ))}
                    </Grid>
                </Box>

                {/* Command Configuration */}
                {commandType && (
                    <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                        <Box className="flex items-center justify-between mb-12">
                            <Typography variant="subtitle1" className="font-semibold">
                                Configure {COMMAND_TEMPLATES[commandType].name}
                            </Typography>
                            <Tooltip title="Toggle Raw JSON Mode">
                                <IconButton
                                    size="small"
                                    onClick={() => setRawMode(!rawMode)}
                                    color={rawMode ? "primary" : "default"}
                                >
                                    <Code />
                                </IconButton>
                            </Tooltip>
                        </Box>

                        {rawMode ? (
                            <TextField
                                fullWidth
                                multiline
                                rows={8}
                                value={commandData}
                                onChange={(e) => setCommandData(e.target.value)}
                                label="Command Data (JSON)"
                                variant="outlined"
                                sx={{ fontFamily: 'monospace' }}
                            />
                        ) : (
                            renderFormFields()
                        )}
                    </Paper>
                )}

                {/* Send Button */}
                <Button
                    fullWidth
                    variant="contained"
                    color="secondary"
                    size="large"
                    startIcon={<SendIcon />}
                    onClick={handleSend}
                    disabled={!selectedRobot || !commandType || !commandData || connectionStatus !== "connected"}
                    sx={{ mt: 2 }}
                >
                    Send Command
                </Button>

                {connectionStatus !== "connected" && (
                    <Typography variant="caption" color="error" sx={{ display: 'block', mt: 1, textAlign: 'center' }}>
                        Not connected to server
                    </Typography>
                )}
            </CardContent>
        </Card>
    );
}

CommandPanel.propTypes = {
    robots: PropTypes.array.isRequired,
    selectedRobot: PropTypes.oneOfType([PropTypes.string, PropTypes.number]),
    onRobotChange: PropTypes.func.isRequired,
    onSendCommand: PropTypes.func.isRequired,
    connectionStatus: PropTypes.string.isRequired,
    userId: PropTypes.number
};

export default CommandPanel;
