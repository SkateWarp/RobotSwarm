/**
 * SwarmControlPanel - Control panel for swarm robot deployment and task assignment
 * Allows selecting robots from database, deploying to Gazebo, and assigning swarm tasks
 */
import { useState, useEffect } from "react";
import PropTypes from "prop-types";
import {
    Box,
    Button,
    Card,
    CardContent,
    Chip,
    Checkbox,
    FormControl,
    FormControlLabel,
    Grid,
    InputLabel,
    MenuItem,
    Paper,
    Select,
    Slider,
    TextField,
    Typography,
    Accordion,
    AccordionSummary,
    AccordionDetails,
    IconButton,
    Tooltip,
    List,
    ListItem,
    ListItemText,
    ListItemIcon,
    ListItemSecondaryAction,
    Divider,
    Alert,
    CircularProgress,
    Badge
} from "@mui/material";
import {
    RocketLaunch,
    Delete,
    PlayArrow,
    Stop,
    ExpandMore as ExpandMoreIcon,
    SmartToy,
    Timeline,
    Category,
    LocalShipping,
    Warning,
    CheckCircle,
    Error as ErrorIcon,
    Refresh,
    Circle
} from "@mui/icons-material";

// Formation types available
const FORMATION_TYPES = [
    { value: 'line', label: 'Line', description: 'Robots line up in a row' },
    { value: 'triangle', label: 'Triangle', description: 'Triangular formation' },
    { value: 'circle', label: 'Circle', description: 'Circular formation' },
    { value: 'square', label: 'Square', description: 'Square grid formation' },
    { value: 'v', label: 'V-Shape', description: 'V-shaped flying formation' },
    { value: 'diamond', label: 'Diamond', description: 'Diamond pattern' }
];

// Leader movement modes
const LEADER_MODES = [
    { value: 'waypoint', label: 'Waypoint', description: 'Follow predefined waypoints' },
    { value: 'manual', label: 'Manual', description: 'Control leader with keyboard/joystick' },
    { value: 'circular', label: 'Circular Path', description: 'Leader follows circular path' },
    { value: 'square', label: 'Square Path', description: 'Leader follows square path' },
    { value: 'figure8', label: 'Figure-8', description: 'Leader follows figure-8 path' },
    { value: 'random', label: 'Random Walk', description: 'Leader moves randomly avoiding obstacles' }
];

// Spawn patterns
const SPAWN_PATTERNS = [
    { value: 'grid', label: 'Grid', description: 'Spawn in grid pattern' },
    { value: 'circle', label: 'Circle', description: 'Spawn in circular pattern' },
    { value: 'line', label: 'Line', description: 'Spawn in a line' }
];

// Task types
const SWARM_TASKS = {
    follow_leader: {
        name: 'Follow Leader',
        icon: <Timeline />,
        color: 'primary',
        description: 'Robots follow a leader in snake-like formation'
    },
    formation: {
        name: 'Formation',
        icon: <Category />,
        color: 'secondary',
        description: 'Robots maintain geometric formations'
    },
    transport: {
        name: 'Collaborative Transport',
        icon: <LocalShipping />,
        color: 'success',
        description: 'Robots work together to transport an object'
    }
};

function SwarmControlPanel({
    robots,
    userId,
    swarmService,
    isConnected,
    isRosConnected,
    swarmStatus,
    onError
}) {
    // State
    const [selectedRobots, setSelectedRobots] = useState([]);
    const [deployedRobots, setDeployedRobots] = useState([]);
    const [isDeploying, setIsDeploying] = useState(false);
    const [spawnPattern, setSpawnPattern] = useState('grid');

    // Task configuration
    const [selectedTask, setSelectedTask] = useState(null);

    // Follow Leader settings
    const [leaderMode, setLeaderMode] = useState('waypoint');
    const [waypoints, setWaypoints] = useState([
        { x: 2.0, y: 0.0 },
        { x: 2.0, y: 2.0 },
        { x: 0.0, y: 2.0 },
        { x: 0.0, y: 0.0 }
    ]);
    const [pathRadius, setPathRadius] = useState(3.0);      // For circular path
    const [pathSideLength, setPathSideLength] = useState(4.0); // For square path

    // Formation settings
    const [formationType, setFormationType] = useState('triangle');
    const [movementMode, setMovementMode] = useState('static');
    const [formationCenter, setFormationCenter] = useState({ x: 0.0, y: 0.0 });
    const [formationSpacing, setFormationSpacing] = useState(1.0);
    const [formationTarget, setFormationTarget] = useState({ x: 3.0, y: 3.0 }); // For moving mode

    // Transport settings
    const [objectPosition, setObjectPosition] = useState({ x: 0.0, y: 0.0 });
    const [targetLocation, setTargetLocation] = useState({ x: 3.0, y: 3.0 });
    const [objectSize, setObjectSize] = useState(0.5); // Diameter

    // Update deployed robots from swarm status
    useEffect(() => {
        if (swarmStatus?.robots) {
            setDeployedRobots(swarmStatus.robots);
        }
    }, [swarmStatus]);

    // Robot selection handlers
    const handleRobotToggle = (robotId) => {
        setSelectedRobots(prev => {
            if (prev.includes(robotId)) {
                return prev.filter(id => id !== robotId);
            } else {
                return [...prev, robotId];
            }
        });
    };

    const handleSelectAll = () => {
        if (selectedRobots.length === robots.length) {
            setSelectedRobots([]);
        } else {
            setSelectedRobots(robots.map(r => r.id));
        }
    };

    // Deploy selected robots to Gazebo
    const handleDeployRobots = async () => {
        if (selectedRobots.length === 0) {
            onError?.('Please select at least one robot to deploy');
            return;
        }

        setIsDeploying(true);
        try {
            // Get robot details for the selected IDs
            const robotDetails = selectedRobots.map(id => {
                const robot = robots.find(r => r.id === id);
                return {
                    id: String(id),
                    name: robot?.name || `robot_${id}`
                };
            });

            // Deploy robots with their database IDs
            await swarmService.spawnRobotsWithIds(robotDetails, spawnPattern);

        } catch (error) {
            console.error('Failed to deploy robots:', error);
            onError?.('Failed to deploy robots: ' + error.message);
        } finally {
            setIsDeploying(false);
        }
    };

    // Delete deployed robots
    const handleDeleteRobots = async (robotIds = null) => {
        try {
            if (robotIds) {
                await swarmService.deleteRobots(robotIds.map(String));
            } else {
                await swarmService.deleteRobots([]);
            }
        } catch (error) {
            console.error('Failed to delete robots:', error);
            onError?.('Failed to delete robots');
        }
    };

    // Start swarm task
    const handleStartTask = async () => {
        if (!selectedTask) {
            onError?.('Please select a task');
            return;
        }

        if (deployedRobots.length === 0) {
            onError?.('No robots deployed. Please deploy robots first.');
            return;
        }

        try {
            switch (selectedTask) {
                case 'follow_leader':
                    // Build path config based on leader mode
                    const pathConfig = {
                        mode: leaderMode,
                        waypoints: leaderMode === 'waypoint' ? waypoints : undefined,
                        radius: leaderMode === 'circular' ? pathRadius : undefined,
                        sideLength: leaderMode === 'square' ? pathSideLength : undefined
                    };
                    await swarmService.startFollowLeader(
                        deployedRobots.length,
                        leaderMode,
                        pathConfig
                    );
                    break;
                case 'formation':
                    // Build formation config with all parameters
                    const formationConfig = {
                        type: formationType,
                        movementMode: movementMode,
                        center: formationCenter,
                        spacing: formationSpacing,
                        target: movementMode === 'moving' ? formationTarget : undefined
                    };
                    await swarmService.startFormation(
                        deployedRobots.length,
                        formationType,
                        movementMode,
                        formationConfig
                    );
                    break;
                case 'transport':
                    // Build transport config with object and target positions
                    const transportConfig = {
                        objectPosition: objectPosition,
                        objectSize: objectSize,
                        targetLocation: targetLocation
                    };
                    await swarmService.startTransport(
                        deployedRobots.length,  // Use all deployed robots - task adapts dynamically
                        targetLocation.x,
                        targetLocation.y,
                        transportConfig
                    );
                    break;
            }
        } catch (error) {
            console.error('Failed to start task:', error);
            onError?.('Failed to start task: ' + error.message);
        }
    };

    // Stop current task
    const handleStopTask = async () => {
        try {
            await swarmService.stopTask();
        } catch (error) {
            console.error('Failed to stop task:', error);
            onError?.('Failed to stop task');
        }
    };

    // Emergency stop
    const handleEmergencyStop = async () => {
        try {
            await swarmService.emergencyStop();
        } catch (error) {
            console.error('Emergency stop failed:', error);
            onError?.('Emergency stop failed');
        }
    };

    // Get robot status color
    const getRobotStatusColor = (status) => {
        switch (status) {
            case 'active': return 'success';
            case 'idle': return 'default';
            case 'avoiding': return 'warning';
            case 'stuck': return 'error';
            case 'error': return 'error';
            default: return 'default';
        }
    };

    // Render task configuration based on selected task
    const renderTaskConfiguration = () => {
        if (!selectedTask) return null;

        switch (selectedTask) {
            case 'follow_leader':
                return (
                    <Box sx={{ mt: 2 }}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                            <InputLabel>Leader Mode</InputLabel>
                            <Select
                                value={leaderMode}
                                onChange={(e) => setLeaderMode(e.target.value)}
                                label="Leader Mode"
                            >
                                {LEADER_MODES.map((mode) => (
                                    <MenuItem key={mode.value} value={mode.value}>
                                        <Box>
                                            <Typography variant="body1">{mode.label}</Typography>
                                            <Typography variant="caption" color="textSecondary">
                                                {mode.description}
                                            </Typography>
                                        </Box>
                                    </MenuItem>
                                ))}
                            </Select>
                        </FormControl>

                        {leaderMode === 'waypoint' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Waypoints (x, y)
                                </Typography>
                                <Grid container spacing={1}>
                                    {waypoints.map((wp, idx) => (
                                        <Grid item xs={6} key={idx}>
                                            <Box sx={{ display: 'flex', gap: 1 }}>
                                                <TextField
                                                    size="small"
                                                    label={`WP${idx + 1} X`}
                                                    type="number"
                                                    value={wp.x}
                                                    onChange={(e) => {
                                                        const newWaypoints = [...waypoints];
                                                        newWaypoints[idx].x = parseFloat(e.target.value);
                                                        setWaypoints(newWaypoints);
                                                    }}
                                                    inputProps={{ step: 0.5 }}
                                                />
                                                <TextField
                                                    size="small"
                                                    label={`WP${idx + 1} Y`}
                                                    type="number"
                                                    value={wp.y}
                                                    onChange={(e) => {
                                                        const newWaypoints = [...waypoints];
                                                        newWaypoints[idx].y = parseFloat(e.target.value);
                                                        setWaypoints(newWaypoints);
                                                    }}
                                                    inputProps={{ step: 0.5 }}
                                                />
                                            </Box>
                                        </Grid>
                                    ))}
                                </Grid>
                            </Paper>
                        )}

                        {leaderMode === 'circular' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Circular Path Settings
                                </Typography>
                                <TextField
                                    fullWidth
                                    label="Path Radius (m)"
                                    type="number"
                                    value={pathRadius}
                                    onChange={(e) => setPathRadius(parseFloat(e.target.value))}
                                    inputProps={{ step: 0.5, min: 0.5, max: 10 }}
                                    helperText="Radius of the circular path"
                                />
                            </Paper>
                        )}

                        {leaderMode === 'square' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Square Path Settings
                                </Typography>
                                <TextField
                                    fullWidth
                                    label="Side Length (m)"
                                    type="number"
                                    value={pathSideLength}
                                    onChange={(e) => setPathSideLength(parseFloat(e.target.value))}
                                    inputProps={{ step: 0.5, min: 1, max: 10 }}
                                    helperText="Length of each side of the square"
                                />
                            </Paper>
                        )}
                    </Box>
                );

            case 'formation':
                return (
                    <Box sx={{ mt: 2 }}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                            <InputLabel>Formation Type</InputLabel>
                            <Select
                                value={formationType}
                                onChange={(e) => setFormationType(e.target.value)}
                                label="Formation Type"
                            >
                                {FORMATION_TYPES.map((type) => (
                                    <MenuItem key={type.value} value={type.value}>
                                        <Box>
                                            <Typography variant="body1">{type.label}</Typography>
                                            <Typography variant="caption" color="textSecondary">
                                                {type.description}
                                            </Typography>
                                        </Box>
                                    </MenuItem>
                                ))}
                            </Select>
                        </FormControl>

                        <FormControl fullWidth sx={{ mb: 2 }}>
                            <InputLabel>Movement Mode</InputLabel>
                            <Select
                                value={movementMode}
                                onChange={(e) => setMovementMode(e.target.value)}
                                label="Movement Mode"
                            >
                                <MenuItem value="static">Static (hold position)</MenuItem>
                                <MenuItem value="moving">Moving (follow leader)</MenuItem>
                                <MenuItem value="adaptive">Adaptive (avoid obstacles)</MenuItem>
                            </Select>
                        </FormControl>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Formation Center Position
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Center X (m)"
                                        type="number"
                                        value={formationCenter.x}
                                        onChange={(e) => setFormationCenter({
                                            ...formationCenter,
                                            x: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Center Y (m)"
                                        type="number"
                                        value={formationCenter.y}
                                        onChange={(e) => setFormationCenter({
                                            ...formationCenter,
                                            y: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                            </Grid>
                        </Paper>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Formation Spacing
                            </Typography>
                            <TextField
                                fullWidth
                                label="Robot Spacing (m)"
                                type="number"
                                value={formationSpacing}
                                onChange={(e) => setFormationSpacing(parseFloat(e.target.value))}
                                inputProps={{ step: 0.1, min: 0.3, max: 5 }}
                                helperText="Distance between robots in formation"
                            />
                        </Paper>

                        {movementMode === 'moving' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Target Position (for moving formation)
                                </Typography>
                                <Grid container spacing={2}>
                                    <Grid item xs={6}>
                                        <TextField
                                            fullWidth
                                            size="small"
                                            label="Target X (m)"
                                            type="number"
                                            value={formationTarget.x}
                                            onChange={(e) => setFormationTarget({
                                                ...formationTarget,
                                                x: parseFloat(e.target.value)
                                            })}
                                            inputProps={{ step: 0.5 }}
                                        />
                                    </Grid>
                                    <Grid item xs={6}>
                                        <TextField
                                            fullWidth
                                            size="small"
                                            label="Target Y (m)"
                                            type="number"
                                            value={formationTarget.y}
                                            onChange={(e) => setFormationTarget({
                                                ...formationTarget,
                                                y: parseFloat(e.target.value)
                                            })}
                                            inputProps={{ step: 0.5 }}
                                        />
                                    </Grid>
                                </Grid>
                            </Paper>
                        )}
                    </Box>
                );

            case 'transport':
                return (
                    <Box sx={{ mt: 2 }}>
                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Object Position (starting location)
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Object X (m)"
                                        type="number"
                                        value={objectPosition.x}
                                        onChange={(e) => setObjectPosition({
                                            ...objectPosition,
                                            x: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Object Y (m)"
                                        type="number"
                                        value={objectPosition.y}
                                        onChange={(e) => setObjectPosition({
                                            ...objectPosition,
                                            y: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                            </Grid>
                        </Paper>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Object Size
                            </Typography>
                            <TextField
                                fullWidth
                                label="Object Diameter (m)"
                                type="number"
                                value={objectSize}
                                onChange={(e) => setObjectSize(parseFloat(e.target.value))}
                                inputProps={{ step: 0.1, min: 0.2, max: 2.0 }}
                                helperText="Diameter of the object to transport"
                            />
                        </Paper>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Target Location (destination)
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Target X (m)"
                                        type="number"
                                        value={targetLocation.x}
                                        onChange={(e) => setTargetLocation({
                                            ...targetLocation,
                                            x: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Target Y (m)"
                                        type="number"
                                        value={targetLocation.y}
                                        onChange={(e) => setTargetLocation({
                                            ...targetLocation,
                                            y: parseFloat(e.target.value)
                                        })}
                                        inputProps={{ step: 0.5 }}
                                    />
                                </Grid>
                            </Grid>
                        </Paper>

                        <Alert severity="info" sx={{ mt: 2 }}>
                            Collaborative transport uses all {deployedRobots.length} deployed robots.
                            Robots will encircle the object and push it to the target location.
                        </Alert>
                    </Box>
                );

            default:
                return null;
        }
    };

    return (
        <Card elevation={3}>
            <CardContent>
                {/* Header */}
                <Box className="flex items-center justify-between mb-16">
                    <Typography variant="h6" className="font-bold">
                        Swarm Control
                    </Typography>
                    <Box className="flex items-center gap-8">
                        <Chip
                            icon={<Circle sx={{ fontSize: 12 }} />}
                            label={isConnected ? "Server Connected" : "Disconnected"}
                            color={isConnected ? "success" : "error"}
                            size="small"
                        />
                        <Chip
                            icon={<Circle sx={{ fontSize: 12 }} />}
                            label={isRosConnected ? "ROS Connected" : "ROS Offline"}
                            color={isRosConnected ? "success" : "warning"}
                            size="small"
                        />
                    </Box>
                </Box>

                {/* Robot Selection Section */}
                <Accordion defaultExpanded>
                    <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                        <Box className="flex items-center gap-8">
                            <SmartToy color="primary" />
                            <Typography variant="subtitle1" className="font-semibold">
                                Robot Selection
                            </Typography>
                            <Chip
                                label={`${selectedRobots.length} selected`}
                                size="small"
                                color="primary"
                            />
                        </Box>
                    </AccordionSummary>
                    <AccordionDetails>
                        <Box className="flex items-center justify-between mb-8">
                            <FormControlLabel
                                control={
                                    <Checkbox
                                        checked={selectedRobots.length === robots.length && robots.length > 0}
                                        indeterminate={selectedRobots.length > 0 && selectedRobots.length < robots.length}
                                        onChange={handleSelectAll}
                                    />
                                }
                                label="Select All"
                            />
                            <FormControl size="small" sx={{ minWidth: 120 }}>
                                <InputLabel>Pattern</InputLabel>
                                <Select
                                    value={spawnPattern}
                                    onChange={(e) => setSpawnPattern(e.target.value)}
                                    label="Pattern"
                                >
                                    {SPAWN_PATTERNS.map((p) => (
                                        <MenuItem key={p.value} value={p.value}>
                                            {p.label}
                                        </MenuItem>
                                    ))}
                                </Select>
                            </FormControl>
                        </Box>

                        <Paper variant="outlined" sx={{ maxHeight: 200, overflow: 'auto' }}>
                            <List dense>
                                {robots.map((robot) => (
                                    <ListItem
                                        key={robot.id}
                                        button
                                        onClick={() => handleRobotToggle(robot.id)}
                                    >
                                        <ListItemIcon>
                                            <Checkbox
                                                edge="start"
                                                checked={selectedRobots.includes(robot.id)}
                                                tabIndex={-1}
                                            />
                                        </ListItemIcon>
                                        <ListItemText
                                            primary={`${robot.name} (ID: ${robot.id})`}
                                            secondary={robot.description || 'No description'}
                                        />
                                        <ListItemSecondaryAction>
                                            <Chip
                                                label={robot.accountId === userId ? "Mine" : "Public"}
                                                size="small"
                                                color={robot.accountId === userId ? "primary" : "default"}
                                            />
                                        </ListItemSecondaryAction>
                                    </ListItem>
                                ))}
                            </List>
                        </Paper>

                        <Box className="flex gap-8 mt-16">
                            <Button
                                variant="contained"
                                color="primary"
                                startIcon={isDeploying ? <CircularProgress size={20} color="inherit" /> : <RocketLaunch />}
                                onClick={handleDeployRobots}
                                disabled={!isConnected || !isRosConnected || selectedRobots.length === 0 || isDeploying}
                                fullWidth
                            >
                                {isDeploying ? 'Deploying...' : `Deploy ${selectedRobots.length} Robot(s)`}
                            </Button>
                            <Button
                                variant="outlined"
                                color="error"
                                startIcon={<Delete />}
                                onClick={() => handleDeleteRobots()}
                                disabled={!isConnected || !isRosConnected || deployedRobots.length === 0}
                            >
                                Delete All
                            </Button>
                        </Box>
                    </AccordionDetails>
                </Accordion>

                {/* Deployed Robots Status */}
                {deployedRobots.length > 0 && (
                    <Accordion defaultExpanded sx={{ mt: 2 }}>
                        <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                            <Box className="flex items-center gap-8">
                                <CheckCircle color="success" />
                                <Typography variant="subtitle1" className="font-semibold">
                                    Deployed Robots
                                </Typography>
                                <Chip
                                    label={`${deployedRobots.length} active`}
                                    size="small"
                                    color="success"
                                />
                            </Box>
                        </AccordionSummary>
                        <AccordionDetails>
                            <Grid container spacing={1}>
                                {deployedRobots.map((robot) => (
                                    <Grid item xs={6} sm={4} key={robot.id}>
                                        <Paper
                                            variant="outlined"
                                            sx={{
                                                p: 1,
                                                textAlign: 'center',
                                                borderColor: `${getRobotStatusColor(robot.status)}.main`
                                            }}
                                        >
                                            <Typography variant="body2" fontWeight="bold">
                                                {robot.id}
                                            </Typography>
                                            <Chip
                                                label={robot.role}
                                                size="small"
                                                color={robot.role === 'leader' ? 'warning' : 'default'}
                                                sx={{ mt: 0.5 }}
                                            />
                                            <Typography variant="caption" display="block" color="textSecondary">
                                                {robot.status}
                                            </Typography>
                                        </Paper>
                                    </Grid>
                                ))}
                            </Grid>
                        </AccordionDetails>
                    </Accordion>
                )}

                <Divider sx={{ my: 2 }} />

                {/* Task Assignment Section */}
                <Accordion defaultExpanded>
                    <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                        <Box className="flex items-center gap-8">
                            <PlayArrow color="secondary" />
                            <Typography variant="subtitle1" className="font-semibold">
                                Task Assignment
                            </Typography>
                            {swarmStatus?.task?.status === 'running' && (
                                <Chip
                                    label={swarmStatus.task.task_type}
                                    size="small"
                                    color="success"
                                />
                            )}
                        </Box>
                    </AccordionSummary>
                    <AccordionDetails>
                        {/* Task Type Selection */}
                        <Grid container spacing={2} sx={{ mb: 2 }}>
                            {Object.entries(SWARM_TASKS).map(([key, task]) => (
                                <Grid item xs={12} sm={4} key={key}>
                                    <Button
                                        fullWidth
                                        variant={selectedTask === key ? "contained" : "outlined"}
                                        color={task.color}
                                        startIcon={task.icon}
                                        onClick={() => setSelectedTask(key)}
                                        disabled={!isConnected || !isRosConnected || deployedRobots.length === 0}
                                        sx={{ py: 1.5 }}
                                    >
                                        {task.name}
                                    </Button>
                                </Grid>
                            ))}
                        </Grid>

                        {selectedTask && (
                            <Alert severity="info" sx={{ mb: 2 }}>
                                {SWARM_TASKS[selectedTask].description}
                            </Alert>
                        )}

                        {/* Task Configuration */}
                        {renderTaskConfiguration()}

                        {/* Task Control Buttons */}
                        <Box className="flex gap-8 mt-16">
                            <Button
                                variant="contained"
                                color="success"
                                startIcon={<PlayArrow />}
                                onClick={handleStartTask}
                                disabled={
                                    !isConnected ||
                                    !isRosConnected ||
                                    !selectedTask ||
                                    deployedRobots.length === 0 ||
                                    swarmStatus?.task?.status === 'running'
                                }
                                fullWidth
                            >
                                Start Task
                            </Button>
                            <Button
                                variant="outlined"
                                color="warning"
                                startIcon={<Stop />}
                                onClick={handleStopTask}
                                disabled={!isConnected || swarmStatus?.task?.status !== 'running'}
                            >
                                Stop
                            </Button>
                        </Box>
                    </AccordionDetails>
                </Accordion>

                {/* Emergency Stop */}
                <Button
                    variant="contained"
                    color="error"
                    startIcon={<Warning />}
                    onClick={handleEmergencyStop}
                    disabled={!isConnected}
                    fullWidth
                    size="large"
                    sx={{ mt: 2 }}
                >
                    EMERGENCY STOP
                </Button>

                {/* Task Status */}
                {swarmStatus?.task && (
                    <Paper variant="outlined" sx={{ p: 2, mt: 2 }}>
                        <Typography variant="subtitle2" gutterBottom>
                            Current Task Status
                        </Typography>
                        <Grid container spacing={2}>
                            <Grid item xs={6}>
                                <Typography variant="caption" color="textSecondary">Type</Typography>
                                <Typography variant="body2">{swarmStatus.task.task_type || 'None'}</Typography>
                            </Grid>
                            <Grid item xs={6}>
                                <Typography variant="caption" color="textSecondary">Status</Typography>
                                <Chip
                                    label={swarmStatus.task.status}
                                    size="small"
                                    color={
                                        swarmStatus.task.status === 'running' ? 'success' :
                                        swarmStatus.task.status === 'failed' ? 'error' : 'default'
                                    }
                                />
                            </Grid>
                            <Grid item xs={12}>
                                <Typography variant="caption" color="textSecondary">Progress</Typography>
                                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                                    <Box sx={{ flex: 1 }}>
                                        <Slider
                                            value={swarmStatus.task.progress * 100}
                                            disabled
                                            valueLabelDisplay="auto"
                                            valueLabelFormat={(v) => `${v.toFixed(0)}%`}
                                        />
                                    </Box>
                                    <Typography variant="body2">
                                        {(swarmStatus.task.progress * 100).toFixed(0)}%
                                    </Typography>
                                </Box>
                            </Grid>
                        </Grid>
                    </Paper>
                )}
            </CardContent>
        </Card>
    );
}

SwarmControlPanel.propTypes = {
    robots: PropTypes.array.isRequired,
    userId: PropTypes.number,
    swarmService: PropTypes.object.isRequired,
    isConnected: PropTypes.bool,
    isRosConnected: PropTypes.bool,
    swarmStatus: PropTypes.object,
    onError: PropTypes.func
};

SwarmControlPanel.defaultProps = {
    isConnected: false,
    isRosConnected: false,
    swarmStatus: null
};

export default SwarmControlPanel;
