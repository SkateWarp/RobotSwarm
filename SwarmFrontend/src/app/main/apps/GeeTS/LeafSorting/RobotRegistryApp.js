import { useCallback, useEffect, useMemo, useState } from "react";
import { useNavigate } from "react-router-dom";
import { Add, DeleteOutline, EditOutlined, Refresh, SmartToyOutlined } from "@mui/icons-material";
import {
    Alert,
    Box,
    Button,
    Card,
    CardContent,
    Chip,
    CircularProgress,
    Dialog,
    DialogActions,
    DialogContent,
    DialogContentText,
    DialogTitle,
    FormControlLabel,
    Grid,
    IconButton,
    Paper,
    Stack,
    Switch,
    TextField,
    Tooltip,
    Typography,
} from "@mui/material";
import {
    createRegistryRobot,
    disableRegistryRobot,
    getRobotRegistryErrorMessage,
    listRegistryRobots,
    updateRegistryRobot,
} from "./robotRegistryApi";
import { normalizeRobotDraft, robotRegistryStatus, validateRobotDraft } from "./robotRegistryModel";

const emptyRobot = {
    id: null,
    name: "",
    description: "",
    notes: "",
    status: 0,
    isPublic: false,
};

function RobotRegistryApp() {
    const navigate = useNavigate();
    const [robots, setRobots] = useState([]);
    const [loading, setLoading] = useState(true);
    const [busy, setBusy] = useState(false);
    const [error, setError] = useState("");
    const [search, setSearch] = useState("");
    const [editor, setEditor] = useState(null);
    const [editorErrors, setEditorErrors] = useState({});
    const [robotToDisable, setRobotToDisable] = useState(null);

    const loadRobots = useCallback(async () => {
        setLoading(true);
        setError("");
        try {
            setRobots(await listRegistryRobots());
        } catch (requestError) {
            setError(getRobotRegistryErrorMessage(requestError));
        } finally {
            setLoading(false);
        }
    }, []);

    useEffect(() => {
        loadRobots();
    }, [loadRobots]);

    const visibleRobots = useMemo(() => {
        const query = search.trim().toLocaleLowerCase("es");
        if (!query) return robots;
        return robots.filter((robot) =>
            `${robot.name} ${robot.description || ""} ${robot.notes || ""} ${robot.namespace || ""}`
                .toLocaleLowerCase("es")
                .includes(query)
        );
    }, [robots, search]);

    const connectedCount = robots.filter((robot) => robot.isConnected).length;

    const runAndRefresh = async (operation) => {
        setBusy(true);
        setError("");
        try {
            await operation();
            await loadRobots();
            return true;
        } catch (requestError) {
            setError(getRobotRegistryErrorMessage(requestError));
            return false;
        } finally {
            setBusy(false);
        }
    };

    const saveRobot = async () => {
        const validation = validateRobotDraft(editor);
        setEditorErrors(validation);
        if (Object.keys(validation).length > 0) return;

        const draft = normalizeRobotDraft(editor);
        const saved = await runAndRefresh(() =>
            editor.id ? updateRegistryRobot(editor.id, draft) : createRegistryRobot(draft)
        );
        if (saved) setEditor(null);
    };

    const disableRobot = async () => {
        const disabled = await runAndRefresh(() => disableRegistryRobot(robotToDisable));
        if (disabled) setRobotToDisable(null);
    };

    return (
        <Box data-testid="robot-registry-page" sx={{ p: { xs: 2, md: 3 }, maxWidth: 1600, mx: "auto" }}>
            <Paper
                elevation={0}
                sx={{
                    p: { xs: 2, md: 3 },
                    mb: 3,
                    borderRadius: 3,
                    color: "common.white",
                    background: "linear-gradient(135deg, #0f172a 0%, #334155 54%, #0f766e 100%)",
                }}
            >
                <Stack direction={{ xs: "column", md: "row" }} spacing={2} justifyContent="space-between">
                    <Box>
                        <Stack direction="row" spacing={1.5} alignItems="center">
                            <SmartToyOutlined />
                            <Typography component="h1" variant="h5" sx={{ fontWeight: 700 }}>
                                Registro de robots
                            </Typography>
                        </Stack>
                        <Typography variant="body2" sx={{ mt: 1, color: "rgba(255,255,255,.84)" }}>
                            Inventario persistente de TurtleBot3. Las instancias activas de Gazebo se
                            supervisan por sesión.
                        </Typography>
                        <Typography variant="caption" sx={{ display: "block", mt: 1 }}>
                            {robots.length} robots activos · {connectedCount} conectados
                        </Typography>
                    </Box>
                    <Stack direction="row" spacing={1} alignItems="center">
                        <Button
                            color="inherit"
                            variant="outlined"
                            onClick={() => navigate("/apps/GTS/realtime")}
                        >
                            Ver robots de sesión
                        </Button>
                        <Button
                            color="inherit"
                            variant="contained"
                            startIcon={<Add />}
                            onClick={() => {
                                setEditor(emptyRobot);
                                setEditorErrors({});
                            }}
                        >
                            Registrar robot
                        </Button>
                    </Stack>
                </Stack>
            </Paper>

            <Paper elevation={1} sx={{ p: 2, mb: 2, borderRadius: 2 }}>
                <Stack direction="row" spacing={1} alignItems="center">
                    <TextField
                        fullWidth
                        size="small"
                        label="Buscar por nombre, descripción, notas o namespace"
                        value={search}
                        onChange={(event) => setSearch(event.target.value)}
                    />
                    <Tooltip title="Actualizar robots">
                        <span>
                            <IconButton
                                aria-label="Actualizar robots"
                                onClick={loadRobots}
                                disabled={loading || busy}
                            >
                                <Refresh />
                            </IconButton>
                        </span>
                    </Tooltip>
                </Stack>
            </Paper>

            {error && (
                <Alert
                    severity="error"
                    aria-live="assertive"
                    sx={{ mb: 2 }}
                    action={
                        <Button color="inherit" size="small" onClick={loadRobots} disabled={busy}>
                            Reintentar
                        </Button>
                    }
                >
                    {error}
                </Alert>
            )}

            {loading && (
                <Box sx={{ py: 8, textAlign: "center" }} role="status" aria-label="Cargando robots">
                    <CircularProgress />
                </Box>
            )}

            {!loading && !error && visibleRobots.length === 0 && (
                <Paper
                    elevation={0}
                    sx={{ p: 5, textAlign: "center", border: "1px dashed", borderColor: "divider" }}
                >
                    <Typography variant="h6">
                        {robots.length === 0 ? "No hay robots activos" : "No hay coincidencias"}
                    </Typography>
                    <Typography color="text.secondary" sx={{ mt: 1 }}>
                        {robots.length === 0
                            ? "Registre el primer robot del inventario."
                            : "Pruebe con otro término de búsqueda."}
                    </Typography>
                </Paper>
            )}

            {!loading && visibleRobots.length > 0 && (
                <Grid container spacing={2}>
                    {visibleRobots.map((robot) => (
                        <Grid item xs={12} md={6} xl={4} key={robot.id}>
                            <Card variant="outlined" sx={{ height: "100%", borderRadius: 2 }}>
                                <CardContent>
                                    <Stack direction="row" justifyContent="space-between" spacing={1}>
                                        <Box sx={{ minWidth: 0 }}>
                                            <Typography variant="h6" noWrap>
                                                {robot.name}
                                            </Typography>
                                            <Typography variant="caption" color="text.secondary">
                                                {robot.namespace || "Sin namespace ROS"}
                                            </Typography>
                                        </Box>
                                        <Stack direction="row">
                                            <Tooltip title="Editar robot">
                                                <span>
                                                    <IconButton
                                                        aria-label={`Editar ${robot.name}`}
                                                        disabled={busy}
                                                        onClick={() => {
                                                            setEditor({ ...robot });
                                                            setEditorErrors({});
                                                        }}
                                                    >
                                                        <EditOutlined />
                                                    </IconButton>
                                                </span>
                                            </Tooltip>
                                            <Tooltip title="Desactivar robot">
                                                <span>
                                                    <IconButton
                                                        aria-label={`Desactivar ${robot.name}`}
                                                        disabled={busy}
                                                        onClick={() => setRobotToDisable(robot)}
                                                    >
                                                        <DeleteOutline />
                                                    </IconButton>
                                                </span>
                                            </Tooltip>
                                        </Stack>
                                    </Stack>
                                    <Stack
                                        direction="row"
                                        spacing={1}
                                        sx={{ my: 2, flexWrap: "wrap", gap: 0.5 }}
                                    >
                                        <Chip size="small" label={robotRegistryStatus(robot)} />
                                        <Chip
                                            size="small"
                                            color={robot.isConnected ? "success" : "default"}
                                            label={robot.isConnected ? "Conectado" : "Sin conexión"}
                                        />
                                        <Chip
                                            size="small"
                                            variant="outlined"
                                            label={robot.isPublic ? "Público" : "Privado"}
                                        />
                                    </Stack>
                                    <Typography variant="body2">
                                        {robot.description || "Sin descripción."}
                                    </Typography>
                                    {robot.notes && (
                                        <Typography
                                            variant="caption"
                                            color="text.secondary"
                                            sx={{ display: "block", mt: 1 }}
                                        >
                                            Notas: {robot.notes}
                                        </Typography>
                                    )}
                                </CardContent>
                            </Card>
                        </Grid>
                    ))}
                </Grid>
            )}

            <Dialog
                open={Boolean(editor)}
                onClose={busy ? undefined : () => setEditor(null)}
                fullWidth
                maxWidth="sm"
            >
                <DialogTitle>{editor?.id ? "Editar robot" : "Registrar robot"}</DialogTitle>
                <DialogContent>
                    <Stack spacing={2} sx={{ pt: 1 }}>
                        <TextField
                            autoFocus
                            required
                            label="Nombre"
                            value={editor?.name || ""}
                            error={Boolean(editorErrors.name)}
                            helperText={editorErrors.name}
                            onChange={(event) =>
                                setEditor((current) => ({ ...current, name: event.target.value }))
                            }
                        />
                        <TextField
                            label="Descripción"
                            value={editor?.description || ""}
                            onChange={(event) =>
                                setEditor((current) => ({ ...current, description: event.target.value }))
                            }
                        />
                        <TextField
                            label="Notas"
                            multiline
                            minRows={3}
                            value={editor?.notes || ""}
                            onChange={(event) =>
                                setEditor((current) => ({ ...current, notes: event.target.value }))
                            }
                        />
                        <FormControlLabel
                            control={
                                <Switch
                                    checked={Boolean(editor?.isPublic)}
                                    onChange={(event) =>
                                        setEditor((current) => ({
                                            ...current,
                                            isPublic: event.target.checked,
                                        }))
                                    }
                                />
                            }
                            label="Visible para otros usuarios"
                        />
                    </Stack>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setEditor(null)} disabled={busy}>
                        Cancelar
                    </Button>
                    <Button variant="contained" onClick={saveRobot} disabled={busy}>
                        {busy ? "Guardando…" : "Guardar"}
                    </Button>
                </DialogActions>
            </Dialog>

            <Dialog
                open={Boolean(robotToDisable)}
                onClose={busy ? undefined : () => setRobotToDisable(null)}
                fullWidth
                maxWidth="xs"
            >
                <DialogTitle>Desactivar robot</DialogTitle>
                <DialogContent>
                    <DialogContentText>
                        “{robotToDisable?.name}” dejará de aparecer como disponible. Esta acción no detiene una
                        sesión ROS activa.
                    </DialogContentText>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setRobotToDisable(null)} disabled={busy}>
                        Cancelar
                    </Button>
                    <Button variant="contained" color="error" onClick={disableRobot} disabled={busy}>
                        {busy ? "Desactivando…" : "Desactivar"}
                    </Button>
                </DialogActions>
            </Dialog>
        </Box>
    );
}

export default RobotRegistryApp;
