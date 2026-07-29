import PropTypes from "prop-types";
import clsx from "clsx";
import { useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import {
    Alert,
    Button,
    Chip,
    Dialog,
    DialogActions,
    DialogContent,
    DialogContentText,
    DialogTitle,
    Icon,
    IconButton,
    MenuItem,
    Paper,
    TextField,
    Tooltip,
    Typography,
} from "@mui/material";
import AccountsTable from "./AccountsTable";
import {
    disableAccount,
    getAccounts,
    openEditAccountDialog,
    reactivateAccount,
    selectAccounts,
    setAccountsSearchText,
} from "./store/accountsSlice";
import { filterAccounts, isCurrentAccount } from "./accountsViewModel";
import { showMessage } from "../../../store/fuse/messageSlice";

const roleLabels = {
    Admin: "Administrador",
    User: "Usuario",
};

const describeStatusChange = (statusChange) => {
    if (!statusChange?.account) {
        return "";
    }

    const { account, enable } = statusChange;
    if (enable) {
        return `${account.firstName} ${account.lastName} podrá iniciar sesión de nuevo. Sus sesiones y visores anteriores permanecerán cerrados.`;
    }

    return `Se impedirá el acceso de ${account.firstName} ${account.lastName} y se cerrarán sus sesiones activas.`;
};

function AccountsList({ embedded, showSearch }) {
    const dispatch = useDispatch();
    const accounts = useSelector(selectAccounts);
    const currentUser = useSelector(({ auth }) => auth.user);
    const { error, loading, searchText } = useSelector(({ accountsApp }) => accountsApp.accounts);
    const [roleFilter, setRoleFilter] = useState("all");
    const [statusFilter, setStatusFilter] = useState("active");
    const [statusChange, setStatusChange] = useState(null);
    const [changingStatus, setChangingStatus] = useState(false);

    const filteredAccounts = useMemo(
        () =>
            filterAccounts(accounts, {
                role: roleFilter,
                status: statusFilter,
                searchText,
            }),
        [accounts, roleFilter, searchText, statusFilter]
    );

    const columns = useMemo(
        () => [
            {
                Header: "Nombre",
                id: "fullName",
                accessor: (account) => `${account.firstName} ${account.lastName}`.trim(),
                sortable: true,
            },
            {
                Header: "Correo electrónico",
                accessor: "email",
                sortable: true,
            },
            {
                Header: "Rol",
                accessor: "role",
                Cell: ({ value }) => roleLabels[value] || value,
                sortable: true,
            },
            {
                Header: "Estado",
                accessor: "enabled",
                Cell: ({ value }) => (
                    <Chip
                        color={value ? "success" : "default"}
                        label={value ? "Activa" : "Inactiva"}
                        size="small"
                        variant={value ? "filled" : "outlined"}
                    />
                ),
                sortable: true,
            },
            {
                id: "actions",
                Header: "Acciones",
                className: "justify-center",
                width: 96,
                sortable: false,
                Cell: ({ row }) => {
                    const account = row.original;
                    const belongsToCurrentUser = isCurrentAccount(account, currentUser);
                    const reactivating = !account.enabled;
                    const disabled = belongsToCurrentUser;
                    const actionLabel = reactivating ? "Reactivar" : "Desactivar";
                    let title = `${actionLabel} cuenta`;
                    if (belongsToCurrentUser) {
                        title = "No puede cambiar el estado de la cuenta con la que inició sesión.";
                    }

                    return (
                        <Tooltip title={title}>
                            <span>
                                <IconButton
                                    aria-label={`${actionLabel} la cuenta de ${account.firstName} ${account.lastName}`}
                                    disabled={disabled}
                                    onClick={(event) => {
                                        event.stopPropagation();
                                        setStatusChange({ account, enable: reactivating });
                                    }}
                                    size="large"
                                >
                                    <Icon>{reactivating ? "person_add" : "person_off"}</Icon>
                                </IconButton>
                            </span>
                        </Tooltip>
                    );
                },
            },
        ],
        [currentUser]
    );

    const closeStatusDialog = () => {
        if (!changingStatus) {
            setStatusChange(null);
        }
    };

    const confirmStatusChange = async () => {
        const account = statusChange?.account;
        if (!account || isCurrentAccount(account, currentUser)) {
            return;
        }

        setChangingStatus(true);
        try {
            const action = statusChange.enable ? reactivateAccount : disableAccount;
            await dispatch(action(account.id)).unwrap();
            dispatch(
                showMessage({
                    message: statusChange.enable ? "Cuenta reactivada." : "Cuenta desactivada.",
                    variant: "success",
                })
            );
            setStatusChange(null);
        } catch (requestError) {
            let errorMessage = statusChange.enable
                ? "No fue posible reactivar la cuenta."
                : "No fue posible desactivar la cuenta.";
            if (typeof requestError === "string") {
                errorMessage = requestError;
            }
            dispatch(
                showMessage({
                    message: errorMessage,
                    variant: "error",
                })
            );
        } finally {
            setChangingStatus(false);
        }
    };

    if (loading && accounts.length === 0) {
        return (
            <div className="flex flex-1 items-center justify-center h-full" role="status">
                <Typography color="textSecondary">Cargando cuentas…</Typography>
            </div>
        );
    }

    const statusActionLabel = statusChange?.enable ? "Reactivar" : "Desactivar";
    const changingStatusLabel = statusChange?.enable ? "Reactivando…" : "Desactivando…";
    const statusButtonLabel = changingStatus ? changingStatusLabel : statusActionLabel;

    return (
        <div
            className={clsx(
                "flex flex-col flex-auto w-full min-h-full gap-12",
                embedded ? "p-0" : "p-8 sm:p-16"
            )}
        >
            <Paper
                className="flex flex-col sm:flex-row sm:items-center gap-12 p-12 sm:p-16"
                variant="outlined"
            >
                <div className="flex flex-1 flex-col sm:flex-row gap-12">
                    {showSearch ? (
                        <TextField
                            fullWidth
                            label="Buscar usuarios"
                            onChange={(event) => dispatch(setAccountsSearchText(event))}
                            size="small"
                            value={searchText}
                        />
                    ) : null}
                    <TextField
                        aria-label="Filtrar cuentas por estado"
                        label="Estado"
                        onChange={(event) => setStatusFilter(event.target.value)}
                        select
                        size="small"
                        value={statusFilter}
                    >
                        <MenuItem value="active">Activas</MenuItem>
                        <MenuItem value="inactive">Inactivas</MenuItem>
                        <MenuItem value="all">Todas</MenuItem>
                    </TextField>
                    <TextField
                        aria-label="Filtrar cuentas por rol"
                        label="Rol"
                        onChange={(event) => setRoleFilter(event.target.value)}
                        select
                        size="small"
                        value={roleFilter}
                    >
                        <MenuItem value="all">Todos</MenuItem>
                        <MenuItem value="User">Usuarios</MenuItem>
                        <MenuItem value="Admin">Administradores</MenuItem>
                    </TextField>
                </div>
                <Typography color="textSecondary" variant="body2" aria-live="polite">
                    {filteredAccounts.length}{" "}
                    {filteredAccounts.length === 1 ? "cuenta encontrada" : "cuentas encontradas"}
                </Typography>
            </Paper>

            {error ? (
                <Alert
                    action={
                        <Button color="inherit" onClick={() => dispatch(getAccounts())} size="small">
                            Reintentar
                        </Button>
                    }
                    severity="error"
                >
                    {error}
                </Alert>
            ) : null}

            {!error && filteredAccounts.length === 0 ? (
                <div className="flex flex-1 flex-col items-center justify-center min-h-256 text-center p-24">
                    <Icon className="text-48 mb-12" color="disabled">
                        manage_accounts
                    </Icon>
                    <Typography variant="h6">No hay cuentas que coincidan</Typography>
                    <Typography color="textSecondary" className="mt-4">
                        Cambie los filtros o el texto de búsqueda para ampliar los resultados.
                    </Typography>
                </div>
            ) : null}

            {filteredAccounts.length > 0 ? (
                <div className="flex flex-auto w-full min-h-0">
                    <AccountsTable
                        columns={columns}
                        data={filteredAccounts}
                        onRowClick={(event, row) => {
                            if (row) {
                                dispatch(openEditAccountDialog(row.original));
                            }
                        }}
                    />
                </div>
            ) : null}

            <Dialog open={!!statusChange} onClose={closeStatusDialog}>
                <DialogTitle>{statusActionLabel} cuenta</DialogTitle>
                <DialogContent>
                    <DialogContentText>{describeStatusChange(statusChange)}</DialogContentText>
                </DialogContent>
                <DialogActions>
                    <Button disabled={changingStatus} onClick={closeStatusDialog}>
                        Cancelar
                    </Button>
                    <Button
                        color={statusChange?.enable ? "primary" : "error"}
                        disabled={changingStatus}
                        onClick={confirmStatusChange}
                        variant="contained"
                    >
                        {statusButtonLabel}
                    </Button>
                </DialogActions>
            </Dialog>
        </div>
    );
}

AccountsList.propTypes = {
    embedded: PropTypes.bool,
    showSearch: PropTypes.bool,
};

AccountsList.defaultProps = {
    embedded: false,
    showSearch: false,
};

export default AccountsList;
