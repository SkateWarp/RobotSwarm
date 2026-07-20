import {
    Alert,
    Button,
    Dialog,
    DialogActions,
    DialogContent,
    DialogTitle,
    MenuItem,
    TextField,
    Typography,
} from "@mui/material";
import { useEffect, useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { showMessage } from "../../../../store/fuse/messageSlice";
import {
    TASK_TEMPLATE_NAME_LIMIT,
    TASK_TEMPLATE_TYPES,
    normalizeTaskTemplateDraft,
    normalizeTaskTemplateType,
    validateTaskTemplateDraft,
} from "./taskTemplateModel";
import { closeTaskTemplateDialog, updateTaskTemplate } from "./store/leafTypeConfigSlice";

const emptyDraft = {
    name: "",
    taskType: "",
};

function LeafTypesConfigDialog() {
    const dispatch = useDispatch();
    const { editDialog, saveError, saving } = useSelector(
        ({ leafTypesConfigApp }) => leafTypesConfigApp.leafTypes
    );
    const [draft, setDraft] = useState(emptyDraft);
    const errors = useMemo(() => validateTaskTemplateDraft(draft), [draft]);
    const isValid = Object.keys(errors).length === 0;

    useEffect(() => {
        if (!editDialog.open || !editDialog.template) {
            return;
        }

        setDraft({
            name: editDialog.template.name || "",
            taskType: normalizeTaskTemplateType(editDialog.template.taskType) ?? "",
        });
    }, [editDialog.open, editDialog.template]);

    const closeDialog = () => {
        if (!saving) {
            dispatch(closeTaskTemplateDialog());
        }
    };

    const submit = async (event) => {
        event.preventDefault();
        if (!editDialog.template || !isValid) {
            return;
        }

        try {
            await dispatch(
                updateTaskTemplate({
                    id: editDialog.template.id,
                    ...normalizeTaskTemplateDraft(draft),
                })
            ).unwrap();
            dispatch(showMessage({ message: "Plantilla actualizada.", variant: "success" }));
            dispatch(closeTaskTemplateDialog());
        } catch (_error) {
            // The request error remains visible inside the dialog so the draft is not lost.
        }
    };

    return (
        <Dialog
            aria-labelledby="task-template-dialog-title"
            fullWidth
            maxWidth="sm"
            onClose={closeDialog}
            open={editDialog.open}
        >
            <form aria-busy={saving} noValidate onSubmit={submit}>
                <DialogTitle id="task-template-dialog-title">Editar plantilla de tarea</DialogTitle>
                <DialogContent className="flex flex-col gap-16 pt-8">
                    <Typography color="textSecondary" variant="body2">
                        Los cambios modifican el catálogo compartido de algoritmos. No crean ni eliminan tareas
                        ejecutadas.
                    </Typography>

                    {saveError ? (
                        <Alert severity="error" role="alert">
                            {saveError}
                        </Alert>
                    ) : null}

                    <TextField
                        autoFocus
                        disabled={saving}
                        error={Boolean(errors.name)}
                        fullWidth
                        helperText={errors.name || `${draft.name.trim().length}/${TASK_TEMPLATE_NAME_LIMIT}`}
                        inputProps={{ maxLength: TASK_TEMPLATE_NAME_LIMIT }}
                        label="Nombre"
                        onChange={(event) => setDraft((current) => ({ ...current, name: event.target.value }))}
                        required
                        value={draft.name}
                    />

                    <TextField
                        disabled={saving}
                        error={Boolean(errors.taskType)}
                        fullWidth
                        helperText={errors.taskType || "Algoritmo asociado a la plantilla."}
                        label="Tipo de tarea"
                        onChange={(event) =>
                            setDraft((current) => ({
                                ...current,
                                taskType: event.target.value,
                            }))
                        }
                        required
                        select
                        value={draft.taskType}
                    >
                        {TASK_TEMPLATE_TYPES.map((option) => (
                            <MenuItem key={option.value} value={option.value}>
                                {option.label}
                            </MenuItem>
                        ))}
                    </TextField>
                </DialogContent>
                <DialogActions className="px-24 pb-24">
                    <Button disabled={saving} onClick={closeDialog}>
                        Cancelar
                    </Button>
                    <Button disabled={saving || !isValid} type="submit" variant="contained">
                        {saving ? "Guardando…" : "Guardar cambios"}
                    </Button>
                </DialogActions>
            </form>
        </Dialog>
    );
}

export default LeafTypesConfigDialog;
