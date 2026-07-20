import { useCallback, useEffect, useMemo, useState } from "react";
import { useNavigate } from "react-router-dom";
import {
    Add,
    DeleteOutline,
    EditOutlined,
    GroupsOutlined,
    Refresh,
    RemoveCircleOutline,
} from "@mui/icons-material";
import {
    Alert,
    Box,
    Button,
    Card,
    CardActions,
    CardContent,
    CardHeader,
    Chip,
    CircularProgress,
    Dialog,
    DialogActions,
    DialogContent,
    DialogContentText,
    DialogTitle,
    FormControl,
    Grid,
    IconButton,
    InputLabel,
    MenuItem,
    Paper,
    Select,
    Stack,
    TextField,
    Tooltip,
    Typography,
} from "@mui/material";
import {
    addRobotToGroup,
    createRobotGroup,
    deleteRobotGroup,
    getRobotGroupErrorMessage,
    listGroupRobots,
    listRobotGroups,
    removeRobotFromGroup,
    updateRobotGroup,
} from "./robotGroupApi";
import {
    buildRobotMemberships,
    countAssignedActiveRobots,
    normalizeRobotGroupDraft,
    robotStatusLabel,
    validateRobotGroupDraft,
} from "./robotGroupModel";

const emptyEditor = { id: null, name: "", description: "" };

const formatDate = (value) => {
    const date = new Date(value);
    if (Number.isNaN(date.getTime())) return "Fecha no disponible";
    return new Intl.DateTimeFormat("es-BO", { dateStyle: "medium" }).format(date);
};

function RobotGroupsApp() {
    const navigate = useNavigate();
    const [groups, setGroups] = useState([]);
    const [robots, setRobots] = useState([]);
    const [loading, setLoading] = useState(true);
    const [busy, setBusy] = useState(false);
    const [error, setError] = useState("");
    const [search, setSearch] = useState("");
    const [editor, setEditor] = useState(null);
    const [editorErrors, setEditorErrors] = useState({});
    const [confirmation, setConfirmation] = useState(null);

    const loadData = useCallback(async () => {
        setLoading(true);
        setError("");
        try {
            const [groupList, robotList] = await Promise.all([listRobotGroups(), listGroupRobots()]);
            setGroups(groupList);
            setRobots(robotList);
        } catch (requestError) {
            setError(getRobotGroupErrorMessage(requestError));
        } finally {
            setLoading(false);
        }
    }, []);

    useEffect(() => {
        loadData();
    }, [loadData]);

    const memberships = useMemo(() => buildRobotMemberships(groups), [groups]);
    const filteredGroups = useMemo(() => {
        const query = search.trim().toLocaleLowerCase("es");
        if (!query) return groups;
        return groups.filter((group) => {
            const memberNames = (group.robots || []).map((robot) => robot.name).join(" ");
            return `${group.name} ${group.description || ""} ${memberNames}`
                .toLocaleLowerCase("es")
                .includes(query);
        });
    }, [groups, search]);

    const assignedRobotCount = countAssignedActiveRobots(memberships, robots);

    const refreshAfter = async (operation) => {
        setBusy(true);
        setError("");
        try {
            await operation();
            await loadData();
            return true;
        } catch (requestError) {
            setError(getRobotGroupErrorMessage(requestError));
            return false;
        } finally {
            setBusy(false);
        }
    };

    const saveGroup = async () => {
        const validation = validateRobotGroupDraft(editor);
        setEditorErrors(validation);
        if (Object.keys(validation).length > 0) return;

        const draft = normalizeRobotGroupDraft(editor);
        const saved = await refreshAfter(() =>
            editor.id ? updateRobotGroup(editor.id, draft) : createRobotGroup(draft)
        );
        if (saved) setEditor(null);
    };

    const selectRobot = async (group, robotId) => {
        if (!robotId) return;
        const robot = robots.find((item) => item.id === Number(robotId));
        if (!robot) return;

        const previousGroup = memberships.get(robot.id);
        if (previousGroup && previousGroup.id !== group.id) {
            setConfirmation({ kind: "transfer", group, robot, previousGroup });
            return;
        }

        await refreshAfter(() => addRobotToGroup(group.id, robot.id));
    };

    const confirmAction = async () => {
        if (!confirmation) return;

        let operation;
        if (confirmation.kind === "delete") {
            operation = () => deleteRobotGroup(confirmation.group.id);
        } else if (confirmation.kind === "remove") {
            operation = () => removeRobotFromGroup(confirmation.group.id, confirmation.robot.id);
        } else {
            operation = () => addRobotToGroup(confirmation.group.id, confirmation.robot.id, true);
        }

        const completed = await refreshAfter(operation);
        if (completed) setConfirmation(null);
    };

    const confirmationCopy = useMemo(() => {
        if (!confirmation) return { title: "", body: "", action: "Confirmar" };
        if (confirmation.kind === "delete") {
            return {
                title: "Eliminar grupo",
                body: `Se eliminará “${confirmation.group.name}”. Sus robots quedarán disponibles y no se borrarán.`,
                action: "Eliminar grupo",
            };
        }
        if (confirmation.kind === "remove") {
            return {
                title: "Quitar robot del grupo",
                body: `“${confirmation.robot.name}” volverá a quedar sin grupo asignado.`,
                action: "Quitar robot",
            };
        }
        return {
            title: "Transferir robot",
            body: `“${confirmation.robot.name}” pertenece a “${confirmation.previousGroup.name}”. ¿Desea moverlo a “${confirmation.group.name}”?`,
            action: "Transferir",
        };
    }, [confirmation]);

    return (
        <Box data-testid="robot-groups-page" sx={{ p: { xs: 2, md: 3 }, maxWidth: 1600, mx: "auto" }}>
            <Paper
                elevation={0}
                sx={{
                    p: { xs: 2, md: 3 },
                    mb: 3,
                    color: "common.white",
                    borderRadius: 3,
                    background: "linear-gradient(135deg, #312e81 0%, #4f46e5 55%, #0891b2 100%)",
                }}
            >
                <Stack direction={{ xs: "column", md: "row" }} spacing={2} justifyContent="space-between">
                    <Box>
                        <Stack direction="row" spacing={1.5} alignItems="center">
                            <GroupsOutlined />
                            <Typography component="h1" variant="h5" sx={{ fontWeight: 700 }}>
                                Grupos de robots
                            </Typography>
                        </Stack>
                        <Typography variant="body2" sx={{ mt: 1, color: "rgba(255,255,255,.84)" }}>
                            Organice el inventario persistente. Las tareas ROS se ejecutan desde el control de
                            simulación.
                        </Typography>
                        <Typography variant="caption" sx={{ display: "block", mt: 1 }}>
                            {groups.length} grupos · {assignedRobotCount} de {robots.length} robots activos
                            asignados
                        </Typography>
                    </Box>
                    <Stack direction="row" spacing={1} alignItems="center">
                        <Button
                            color="inherit"
                            variant="outlined"
                            onClick={() => navigate("/apps/GTS/realtime")}
                        >
                            Abrir control ROS
                        </Button>
                        <Button
                            color="inherit"
                            variant="contained"
                            startIcon={<Add />}
                            onClick={() => {
                                setEditor(emptyEditor);
                                setEditorErrors({});
                            }}
                        >
                            Crear grupo
                        </Button>
                    </Stack>
                </Stack>
            </Paper>

            <Paper elevation={1} sx={{ p: 2, mb: 2, borderRadius: 2 }}>
                <Stack direction="row" spacing={1} alignItems="center">
                    <TextField
                        fullWidth
                        size="small"
                        label="Buscar por grupo, descripción o robot"
                        value={search}
                        onChange={(event) => setSearch(event.target.value)}
                    />
                    <Tooltip title="Actualizar grupos">
                        <span>
                            <IconButton
                                aria-label="Actualizar grupos"
                                onClick={loadData}
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
                        <Button color="inherit" size="small" onClick={loadData} disabled={busy}>
                            Reintentar
                        </Button>
                    }
                >
                    {error}
                </Alert>
            )}

            {loading && (
                <Box sx={{ py: 8, textAlign: "center" }} role="status" aria-label="Cargando grupos">
                    <CircularProgress />
                </Box>
            )}
            {!loading && !error && filteredGroups.length === 0 && (
                <Paper
                    elevation={0}
                    sx={{ p: 5, textAlign: "center", border: "1px dashed", borderColor: "divider" }}
                >
                    <Typography variant="h6">
                        {groups.length === 0 ? "Todavía no hay grupos" : "No hay coincidencias"}
                    </Typography>
                    <Typography color="text.secondary" sx={{ mt: 1 }}>
                        {groups.length === 0
                            ? "Cree el primer grupo y añada robots del inventario."
                            : "Cambie el texto de búsqueda para ver otros grupos."}
                    </Typography>
                </Paper>
            )}
            {!loading && filteredGroups.length > 0 && (
                <Grid container spacing={2}>
                    {filteredGroups.map((group) => {
                        const candidates = robots.filter(
                            (robot) => memberships.get(robot.id)?.id !== group.id
                        );
                        return (
                            <Grid item xs={12} lg={6} key={group.id}>
                                <Card variant="outlined" sx={{ height: "100%", borderRadius: 2 }}>
                                    <CardHeader
                                        title={group.name}
                                        subheader={`${
                                            (group.robots || []).length
                                        } robots · creado ${formatDate(group.dateCreated)}`}
                                        action={
                                            <Stack direction="row">
                                                <Tooltip title="Editar grupo">
                                                    <span>
                                                        <IconButton
                                                            aria-label={`Editar ${group.name}`}
                                                            disabled={busy}
                                                            onClick={() => {
                                                                setEditor({
                                                                    id: group.id,
                                                                    name: group.name,
                                                                    description: group.description || "",
                                                                });
                                                                setEditorErrors({});
                                                            }}
                                                        >
                                                            <EditOutlined />
                                                        </IconButton>
                                                    </span>
                                                </Tooltip>
                                                <Tooltip title="Eliminar grupo">
                                                    <span>
                                                        <IconButton
                                                            aria-label={`Eliminar ${group.name}`}
                                                            disabled={busy}
                                                            onClick={() =>
                                                                setConfirmation({ kind: "delete", group })
                                                            }
                                                        >
                                                            <DeleteOutline />
                                                        </IconButton>
                                                    </span>
                                                </Tooltip>
                                            </Stack>
                                        }
                                    />
                                    <CardContent sx={{ pt: 0 }}>
                                        <Typography color="text.secondary" sx={{ minHeight: 40, mb: 2 }}>
                                            {group.description || "Sin descripción."}
                                        </Typography>

                                        {(group.robots || []).length === 0 ? (
                                            <Alert severity="info" icon={false} sx={{ mb: 2 }}>
                                                Este grupo todavía no tiene robots.
                                            </Alert>
                                        ) : (
                                            <Stack spacing={1} sx={{ mb: 2 }}>
                                                {(group.robots || []).map((robot) => (
                                                    <Paper
                                                        key={robot.id}
                                                        variant="outlined"
                                                        sx={{
                                                            p: 1,
                                                            display: "flex",
                                                            alignItems: "center",
                                                            gap: 1,
                                                        }}
                                                    >
                                                        <Box sx={{ minWidth: 0, flexGrow: 1 }}>
                                                            <Typography
                                                                variant="body2"
                                                                sx={{ fontWeight: 600 }}
                                                                noWrap
                                                            >
                                                                {robot.name}
                                                            </Typography>
                                                            <Typography
                                                                variant="caption"
                                                                color="text.secondary"
                                                            >
                                                                {robot.namespace || "Sin namespace ROS"} ·{" "}
                                                                {robotStatusLabel(robot)}
                                                            </Typography>
                                                        </Box>
                                                        <Chip
                                                            size="small"
                                                            color={robot.isConnected ? "success" : "default"}
                                                            label={
                                                                robot.isConnected
                                                                    ? "Conectado"
                                                                    : "Sin conexión"
                                                            }
                                                        />
                                                        <Tooltip title="Quitar del grupo">
                                                            <span>
                                                                <IconButton
                                                                    size="small"
                                                                    aria-label={`Quitar ${robot.name} de ${group.name}`}
                                                                    disabled={busy}
                                                                    onClick={() =>
                                                                        setConfirmation({
                                                                            kind: "remove",
                                                                            group,
                                                                            robot,
                                                                        })
                                                                    }
                                                                >
                                                                    <RemoveCircleOutline fontSize="small" />
                                                                </IconButton>
                                                            </span>
                                                        </Tooltip>
                                                    </Paper>
                                                ))}
                                            </Stack>
                                        )}

                                        <FormControl
                                            fullWidth
                                            size="small"
                                            disabled={busy || candidates.length === 0}
                                        >
                                            <InputLabel id={`add-robot-${group.id}`}>Añadir robot</InputLabel>
                                            <Select
                                                labelId={`add-robot-${group.id}`}
                                                value=""
                                                label="Añadir robot"
                                                onChange={(event) => selectRobot(group, event.target.value)}
                                            >
                                                {candidates.map((robot) => {
                                                    const currentGroup = memberships.get(robot.id);
                                                    return (
                                                        <MenuItem key={robot.id} value={robot.id}>
                                                            {robot.name}
                                                            {currentGroup
                                                                ? ` · transferir desde ${currentGroup.name}`
                                                                : ""}
                                                        </MenuItem>
                                                    );
                                                })}
                                            </Select>
                                        </FormControl>
                                    </CardContent>
                                    <CardActions sx={{ px: 2, pt: 0 }}>
                                        <Typography variant="caption" color="text.secondary">
                                            La membresía no inicia ni detiene procesos ROS.
                                        </Typography>
                                    </CardActions>
                                </Card>
                            </Grid>
                        );
                    })}
                </Grid>
            )}

            <Dialog
                open={Boolean(editor)}
                onClose={busy ? undefined : () => setEditor(null)}
                fullWidth
                maxWidth="sm"
            >
                <DialogTitle>{editor?.id ? "Editar grupo" : "Crear grupo"}</DialogTitle>
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
                            multiline
                            minRows={3}
                            value={editor?.description || ""}
                            onChange={(event) =>
                                setEditor((current) => ({ ...current, description: event.target.value }))
                            }
                        />
                    </Stack>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setEditor(null)} disabled={busy}>
                        Cancelar
                    </Button>
                    <Button variant="contained" onClick={saveGroup} disabled={busy}>
                        {busy ? "Guardando…" : "Guardar"}
                    </Button>
                </DialogActions>
            </Dialog>

            <Dialog
                open={Boolean(confirmation)}
                onClose={busy ? undefined : () => setConfirmation(null)}
                fullWidth
                maxWidth="xs"
            >
                <DialogTitle>{confirmationCopy.title}</DialogTitle>
                <DialogContent>
                    <DialogContentText>{confirmationCopy.body}</DialogContentText>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setConfirmation(null)} disabled={busy}>
                        Cancelar
                    </Button>
                    <Button
                        variant="contained"
                        color={confirmation?.kind === "delete" ? "error" : "primary"}
                        onClick={confirmAction}
                        disabled={busy}
                    >
                        {busy ? "Aplicando…" : confirmationCopy.action}
                    </Button>
                </DialogActions>
            </Dialog>
        </Box>
    );
}

export default RobotGroupsApp;
