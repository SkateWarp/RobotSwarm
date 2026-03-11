/**
 * SwarmControlPanel - Panel de control para despliegue de robots y asignación de tareas
 * Permite seleccionar robots de la base de datos, desplegarlos en Gazebo y asignar tareas de enjambre
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

// Tipos de formación disponibles
const FORMATION_TYPES = [
    { value: 'line', label: 'Línea', description: 'Los robots se alinean en fila' },
    { value: 'triangle', label: 'Triángulo', description: 'Formación triangular' },
    { value: 'circle', label: 'Círculo', description: 'Formación circular' },
    { value: 'square', label: 'Cuadrado', description: 'Formación en cuadrícula' },
    { value: 'v', label: 'V', description: 'Formación en V' },
    { value: 'diamond', label: 'Diamante', description: 'Formación en diamante' }
];

// Modos de movimiento del líder
const LEADER_MODES = [
    { value: 'waypoint', label: 'Puntos de ruta', description: 'Seguir puntos de ruta predefinidos' },
    { value: 'manual', label: 'Manual', description: 'Controlar al líder manualmente' },
    { value: 'circular', label: 'Trayectoria circular', description: 'El líder sigue una trayectoria circular' },
    { value: 'square', label: 'Trayectoria cuadrada', description: 'El líder sigue una trayectoria cuadrada' },
    { value: 'figure8', label: 'Figura 8', description: 'El líder sigue una trayectoria en forma de 8' },
    { value: 'random', label: 'Exploración aleatoria', description: 'El líder se mueve aleatoriamente evitando obstáculos' }
];

// Patrones de despliegue
const SPAWN_PATTERNS = [
    { value: 'grid', label: 'Cuadrícula', description: 'Desplegar en cuadrícula' },
    { value: 'circle', label: 'Círculo', description: 'Desplegar en círculo' },
    { value: 'line', label: 'Línea', description: 'Desplegar en línea' }
];

// Tipos de tareas
const SWARM_TASKS = {
    follow_leader: {
        name: 'Seguir al Líder',
        icon: <Timeline />,
        color: 'primary',
        description: 'Los robots siguen a un líder en formación tipo serpiente'
    },
    formation: {
        name: 'Formación',
        icon: <Category />,
        color: 'secondary',
        description: 'Los robots mantienen formaciones geométricas'
    },
    transport: {
        name: 'Transporte Colaborativo',
        icon: <LocalShipping />,
        color: 'success',
        description: 'Los robots trabajan juntos para transportar un objeto'
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
    // Estado
    const [selectedRobots, setSelectedRobots] = useState([]);
    const [deployedRobots, setDeployedRobots] = useState([]);
    const [isDeploying, setIsDeploying] = useState(false);
    const [spawnPattern, setSpawnPattern] = useState('grid');

    // Configuración de tareas
    const [selectedTask, setSelectedTask] = useState(null);

    // Configuración Seguir al Líder
    const [leaderMode, setLeaderMode] = useState('waypoint');
    const [waypoints, setWaypoints] = useState([
        { x: 2.0, y: 0.0 },
        { x: 2.0, y: 2.0 },
        { x: 0.0, y: 2.0 },
        { x: 0.0, y: 0.0 }
    ]);
    const [pathRadius, setPathRadius] = useState(3.0);
    const [pathSideLength, setPathSideLength] = useState(4.0);

    // Configuración de Formación
    const [formationType, setFormationType] = useState('triangle');
    const [movementMode, setMovementMode] = useState('static');
    const [formationCenter, setFormationCenter] = useState({ x: 0.0, y: 0.0 });
    const [formationSpacing, setFormationSpacing] = useState(1.0);
    const [formationTarget, setFormationTarget] = useState({ x: 3.0, y: 3.0 });

    // Configuración de Transporte
    const [objectPosition, setObjectPosition] = useState({ x: 0.0, y: 0.0 });
    const [targetLocation, setTargetLocation] = useState({ x: 3.0, y: 3.0 });
    const [objectSize, setObjectSize] = useState(0.5);

    // Actualizar robots desplegados desde el estado del enjambre
    useEffect(() => {
        if (swarmStatus?.robots) {
            setDeployedRobots(swarmStatus.robots);
        }
    }, [swarmStatus]);

    // Manejadores de selección
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

    // Desplegar robots seleccionados en Gazebo
    const handleDeployRobots = async () => {
        if (selectedRobots.length === 0) {
            onError?.('Selecciona al menos un robot para desplegar');
            return;
        }

        setIsDeploying(true);
        try {
            const robotDetails = selectedRobots.map(id => {
                const robot = robots.find(r => r.id === id);
                return {
                    id: String(id),
                    name: robot?.name || `robot_${id}`
                };
            });

            await swarmService.spawnRobotsWithIds(robotDetails, spawnPattern);
        } catch (error) {
            console.error('Error al desplegar robots:', error);
            onError?.('Error al desplegar robots: ' + error.message);
        } finally {
            setIsDeploying(false);
        }
    };

    // Eliminar robots desplegados
    const handleDeleteRobots = async (robotIds = null) => {
        try {
            if (robotIds) {
                await swarmService.deleteRobots(robotIds.map(String));
            } else {
                await swarmService.deleteRobots([]);
            }
        } catch (error) {
            console.error('Error al eliminar robots:', error);
            onError?.('Error al eliminar robots');
        }
    };

    // Iniciar tarea de enjambre
    const handleStartTask = async () => {
        if (!selectedTask) {
            onError?.('Selecciona una tarea');
            return;
        }

        if (deployedRobots.length === 0) {
            onError?.('No hay robots desplegados. Despliega robots primero.');
            return;
        }

        try {
            switch (selectedTask) {
                case 'follow_leader':
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
                    const transportConfig = {
                        objectPosition: objectPosition,
                        objectSize: objectSize,
                        targetLocation: targetLocation
                    };
                    await swarmService.startTransport(
                        deployedRobots.length,
                        targetLocation.x,
                        targetLocation.y,
                        transportConfig
                    );
                    break;
            }
        } catch (error) {
            console.error('Error al iniciar tarea:', error);
            onError?.('Error al iniciar tarea: ' + error.message);
        }
    };

    // Detener tarea actual
    const handleStopTask = async () => {
        try {
            await swarmService.stopTask();
        } catch (error) {
            console.error('Error al detener tarea:', error);
            onError?.('Error al detener tarea');
        }
    };

    // Parada de emergencia
    const handleEmergencyStop = async () => {
        try {
            await swarmService.emergencyStop();
        } catch (error) {
            console.error('Error en parada de emergencia:', error);
            onError?.('Error en parada de emergencia');
        }
    };

    // Color de estado del robot
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

    // Renderizar configuración según tarea seleccionada
    const renderTaskConfiguration = () => {
        if (!selectedTask) return null;

        switch (selectedTask) {
            case 'follow_leader':
                return (
                    <Box sx={{ mt: 2 }}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                            <InputLabel>Modo del Líder</InputLabel>
                            <Select
                                value={leaderMode}
                                onChange={(e) => setLeaderMode(e.target.value)}
                                label="Modo del Líder"
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
                                    Puntos de ruta (x, y)
                                </Typography>
                                <Grid container spacing={1}>
                                    {waypoints.map((wp, idx) => (
                                        <Grid item xs={6} key={idx}>
                                            <Box sx={{ display: 'flex', gap: 1 }}>
                                                <TextField
                                                    size="small"
                                                    label={`PR${idx + 1} X`}
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
                                                    label={`PR${idx + 1} Y`}
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
                                    Configuración de Trayectoria Circular
                                </Typography>
                                <TextField
                                    fullWidth
                                    label="Radio de trayectoria (m)"
                                    type="number"
                                    value={pathRadius}
                                    onChange={(e) => setPathRadius(parseFloat(e.target.value))}
                                    inputProps={{ step: 0.5, min: 0.5, max: 10 }}
                                    helperText="Radio de la trayectoria circular"
                                />
                            </Paper>
                        )}

                        {leaderMode === 'square' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Configuración de Trayectoria Cuadrada
                                </Typography>
                                <TextField
                                    fullWidth
                                    label="Longitud del lado (m)"
                                    type="number"
                                    value={pathSideLength}
                                    onChange={(e) => setPathSideLength(parseFloat(e.target.value))}
                                    inputProps={{ step: 0.5, min: 1, max: 10 }}
                                    helperText="Longitud de cada lado del cuadrado"
                                />
                            </Paper>
                        )}
                    </Box>
                );

            case 'formation':
                return (
                    <Box sx={{ mt: 2 }}>
                        <FormControl fullWidth sx={{ mb: 2 }}>
                            <InputLabel>Tipo de Formación</InputLabel>
                            <Select
                                value={formationType}
                                onChange={(e) => setFormationType(e.target.value)}
                                label="Tipo de Formación"
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
                            <InputLabel>Modo de Movimiento</InputLabel>
                            <Select
                                value={movementMode}
                                onChange={(e) => setMovementMode(e.target.value)}
                                label="Modo de Movimiento"
                            >
                                <MenuItem value="static">Estático (mantener posición)</MenuItem>
                                <MenuItem value="moving">En movimiento (seguir al líder)</MenuItem>
                                <MenuItem value="adaptive">Adaptativo (evitar obstáculos)</MenuItem>
                            </Select>
                        </FormControl>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Posición Central de la Formación
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Centro X (m)"
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
                                        label="Centro Y (m)"
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
                                Espaciado de la Formación
                            </Typography>
                            <TextField
                                fullWidth
                                label="Distancia entre robots (m)"
                                type="number"
                                value={formationSpacing}
                                onChange={(e) => setFormationSpacing(parseFloat(e.target.value))}
                                inputProps={{ step: 0.1, min: 0.3, max: 5 }}
                                helperText="Distancia entre robots en la formación"
                            />
                        </Paper>

                        {movementMode === 'moving' && (
                            <Paper variant="outlined" sx={{ p: 2 }}>
                                <Typography variant="subtitle2" gutterBottom>
                                    Posición Destino (para formación en movimiento)
                                </Typography>
                                <Grid container spacing={2}>
                                    <Grid item xs={6}>
                                        <TextField
                                            fullWidth
                                            size="small"
                                            label="Destino X (m)"
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
                                            label="Destino Y (m)"
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
                                Posición del Objeto (ubicación inicial)
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Objeto X (m)"
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
                                        label="Objeto Y (m)"
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
                                Tamaño del Objeto
                            </Typography>
                            <TextField
                                fullWidth
                                label="Diámetro del objeto (m)"
                                type="number"
                                value={objectSize}
                                onChange={(e) => setObjectSize(parseFloat(e.target.value))}
                                inputProps={{ step: 0.1, min: 0.2, max: 2.0 }}
                                helperText="Diámetro del objeto a transportar"
                            />
                        </Paper>

                        <Paper variant="outlined" sx={{ p: 2, mb: 2 }}>
                            <Typography variant="subtitle2" gutterBottom>
                                Ubicación Destino
                            </Typography>
                            <Grid container spacing={2}>
                                <Grid item xs={6}>
                                    <TextField
                                        fullWidth
                                        size="small"
                                        label="Destino X (m)"
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
                                        label="Destino Y (m)"
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
                            El transporte colaborativo usa los {deployedRobots.length} robots desplegados.
                            Los robots rodearán el objeto y lo empujarán hacia la ubicación destino.
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
                {/* Encabezado */}
                <Box className="flex items-center justify-between mb-16">
                    <Typography variant="h6" className="font-bold">
                        Control del Enjambre
                    </Typography>
                    <Box className="flex items-center gap-8">
                        <Chip
                            icon={<Circle sx={{ fontSize: 12 }} />}
                            label={isConnected ? "Servidor Conectado" : "Desconectado"}
                            color={isConnected ? "success" : "error"}
                            size="small"
                        />
                        <Chip
                            icon={<Circle sx={{ fontSize: 12 }} />}
                            label={isRosConnected ? "ROS Conectado" : "ROS Sin Conexión"}
                            color={isRosConnected ? "success" : "warning"}
                            size="small"
                        />
                    </Box>
                </Box>

                {/* Sección de Selección de Robots */}
                <Accordion defaultExpanded>
                    <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                        <Box className="flex items-center gap-8">
                            <SmartToy color="primary" />
                            <Typography variant="subtitle1" className="font-semibold">
                                Selección de Robots
                            </Typography>
                            <Chip
                                label={`${selectedRobots.length} seleccionados`}
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
                                label="Seleccionar Todos"
                            />
                            <FormControl size="small" sx={{ minWidth: 120 }}>
                                <InputLabel>Patrón</InputLabel>
                                <Select
                                    value={spawnPattern}
                                    onChange={(e) => setSpawnPattern(e.target.value)}
                                    label="Patrón"
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
                                            secondary={robot.description || 'Sin descripción'}
                                        />
                                        <ListItemSecondaryAction>
                                            <Chip
                                                label={robot.accountId === userId ? "Mío" : "Público"}
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
                                {isDeploying ? 'Desplegando...' : `Desplegar ${selectedRobots.length} Robot(s)`}
                            </Button>
                            <Button
                                variant="outlined"
                                color="error"
                                startIcon={<Delete />}
                                onClick={() => handleDeleteRobots()}
                                disabled={!isConnected || !isRosConnected || deployedRobots.length === 0}
                            >
                                Eliminar
                            </Button>
                        </Box>
                    </AccordionDetails>
                </Accordion>

                {/* Estado de Robots Desplegados */}
                {deployedRobots.length > 0 && (
                    <Accordion defaultExpanded sx={{ mt: 2 }}>
                        <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                            <Box className="flex items-center gap-8">
                                <CheckCircle color="success" />
                                <Typography variant="subtitle1" className="font-semibold">
                                    Robots Desplegados
                                </Typography>
                                <Chip
                                    label={`${deployedRobots.length} activos`}
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

                {/* Sección de Asignación de Tareas */}
                <Accordion defaultExpanded>
                    <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                        <Box className="flex items-center gap-8">
                            <PlayArrow color="secondary" />
                            <Typography variant="subtitle1" className="font-semibold">
                                Asignación de Tareas
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
                        {/* Selección de Tipo de Tarea */}
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

                        {/* Configuración de Tarea */}
                        {renderTaskConfiguration()}

                        {/* Botones de Control de Tarea */}
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
                                Iniciar Tarea
                            </Button>
                            <Button
                                variant="outlined"
                                color="warning"
                                startIcon={<Stop />}
                                onClick={handleStopTask}
                                disabled={!isConnected || !isRosConnected || swarmStatus?.task?.status !== 'running'}
                            >
                                Detener
                            </Button>
                        </Box>
                    </AccordionDetails>
                </Accordion>

                {/* Parada de Emergencia */}
                <Button
                    variant="contained"
                    color="error"
                    startIcon={<Warning />}
                    onClick={handleEmergencyStop}
                    disabled={!isConnected || !isRosConnected}
                    fullWidth
                    size="large"
                    sx={{ mt: 2 }}
                >
                    PARADA DE EMERGENCIA
                </Button>

                {/* Estado de la Tarea */}
                {swarmStatus?.task && (
                    <Paper variant="outlined" sx={{ p: 2, mt: 2 }}>
                        <Typography variant="subtitle2" gutterBottom>
                            Estado de la Tarea Actual
                        </Typography>
                        <Grid container spacing={2}>
                            <Grid item xs={6}>
                                <Typography variant="caption" color="textSecondary">Tipo</Typography>
                                <Typography variant="body2">{swarmStatus.task.task_type || 'Ninguna'}</Typography>
                            </Grid>
                            <Grid item xs={6}>
                                <Typography variant="caption" color="textSecondary">Estado</Typography>
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
                                <Typography variant="caption" color="textSecondary">Progreso</Typography>
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
