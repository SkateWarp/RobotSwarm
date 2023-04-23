/* eslint-disable react-hooks/exhaustive-deps */
import { motion } from "framer-motion";
import { useEffect, useState, useCallback, useMemo } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useParams } from "react-router-dom";
import { Icon, IconButton, Typography } from "@mui/material";
import AccountsTable from "./AccountsTable";
import { getAccounts, openEditAccountDialog, removeAccount, selectAccounts } from "./store/accountsSlice";
import settingsConfig from "../../../fuse-configs/settingsConfig";
import ChargingProgressBar from "../../../shared-components/ChargingProgressBar";
import DeleteRowElementDialog from "../../../shared-components/DeleteRowElementDialog";

function AccountsList() {
    const routeParams = useParams();
    const dispatch = useDispatch();
    const accounts = useSelector(selectAccounts);
    const searchText = useSelector(({ accountsApp }) => accountsApp.accounts.searchText);
    const paginationData = useSelector(({ accountsApp }) => accountsApp.accounts.pagination);

    const [previousSearch, setPreviousSearch] = useState("");
    const [idForDelete, setIdForDelete] = useState(null);
    const [statusFilter, setStatusFilter] = useState(false);
    const [filteredData, setFilteredData] = useState(null);
    const [open, setOpen] = useState(false);

    const handleClickOpen = () => {
        setOpen(true);
    };

    const deleteRowData = () => {
        dispatch(removeAccount({ idForDelete, ...routeParams, searchFilter: searchText, pageNumber: 1 }));
    };

    const columns = useMemo(
        () => [
            {
                Header: "Nombre",
                accessor: "firstName",
                sortable: true,
            },
            {
                Header: "Apellido",
                accessor: "lastName",
                sortable: true,
            },
            {
                Header: "Rol",
                accessor: "role",
                Cell: ({ row }) => {
                    if (settingsConfig.layout.project === "fraga") {
                        if (row.original.role === "User") {
                            return <div>Usuario</div>;
                        }
                        if (row.original.role === "Operator") {
                            return <div>Operador</div>;
                        }
                        if (row.original.role === "Maintenance") {
                            return <div>Mantenimiento</div>;
                        }
                        if (row.original.role === "Reception") {
                            return <div>Recepción</div>;
                        }
                        if (row.original.role === "Supervisor") {
                            return <div>Supervisor</div>;
                        }
                        return <div>{row.original.role}</div>;
                    }
                    if (
                        settingsConfig.layout.project === "baldom" ||
                        settingsConfig.layout.project === "task" ||
                        settingsConfig.layout.project === "panelTemp"
                    ) {
                        if (row.original.role === "User") {
                            return <div>Usuario</div>;
                        }
                        if (row.original.role === "Operator") {
                            return <div>Operador</div>;
                        }
                        if (row.original.role === "Maintenance") {
                            return <div>Mantenimiento</div>;
                        }
                        if (row.original.role === "WasteOperator") {
                            return <div>Operador de Desperdicios</div>;
                        }
                        if (row.original.role === "CleanOperator") {
                            return <div>Operador de Limpieza</div>;
                        }
                        return <div>{row.original.role}</div>;
                    }
                },
                sortable: true,
            },
            {
                Header: "Correo Electrónico",
                accessor: "email",
                sortable: true,
            },
            {
                Header: "Activar",
                accessor: "enabled",
                className: "justify-center",
                Cell: ({ row }) => {
                    return <Icon>{row.original.enabled ? "check" : "clear"}</Icon>;
                },
                sortable: true,
            },
            {
                id: "action",
                Header: "Eliminar",
                className: "justify-center",
                width: 128,
                sortable: false,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        <IconButton
                            onClick={(ev) => {
                                ev.stopPropagation();
                                handleClickOpen();
                                setIdForDelete(row.original.id);
                            }}
                        >
                            <Icon>delete</Icon>
                        </IconButton>
                    </div>
                ),
            },
        ],
        []
    );

    const handleSort = useCallback(
        (sort) => {
            if (sort.length > 0) {
                dispatch(
                    getAccounts({
                        ...routeParams,
                        searchFilter: searchText,
                        pageNumber: 1,
                        pageSize: paginationData.pageSize,
                        sortColumn: sort[0].id,
                        sortDesc: sort[0].desc,
                    })
                );
            }
        },
        [accounts]
    );

    useEffect(() => {
        if (accounts) {
            if (previousSearch !== searchText && (searchText.length >= 3 || searchText === "")) {
                dispatch(
                    getAccounts({
                        ...routeParams,
                        searchFilter: searchText,
                        pageNumber: 1,
                        pageSize: paginationData.pageSize,
                        sortColumn: paginationData.sortColumn,
                        sortDesc: paginationData.sortDesc,
                    })
                );
                setPreviousSearch(searchText);
            }

            setFilteredData(accounts);

            if (accounts.length) {
                setStatusFilter(true);
            } else if (filteredData !== null) {
                setStatusFilter(true);
            } else {
                setStatusFilter(false);
            }
        }
    }, [accounts, searchText, routeParams, previousSearch]);

    if (statusFilter && !filteredData.length) {
        return (
            <div className="flex flex-1 items-center justify-center h-full">
                <Typography color="textSecondary" variant="h5">
                    No existen cuentas!
                </Typography>
            </div>
        );
    }
    if (!statusFilter) {
        return <ChargingProgressBar />;
    }

    return (
        <>
            <motion.div
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                className="flex flex-auto w-full max-h-full"
            >
                <AccountsTable
                    onSort={handleSort}
                    columns={columns}
                    data={filteredData}
                    pagination={paginationData}
                    onRowClick={(ev, row) => {
                        if (row) {
                            dispatch(openEditAccountDialog(row.original));
                        }
                    }}
                />
            </motion.div>

            <DeleteRowElementDialog
                message="este usuario"
                setIsOpen={setOpen}
                deleteData={deleteRowData}
                isOpen={open}
            />
        </>
    );
}

export default AccountsList;
