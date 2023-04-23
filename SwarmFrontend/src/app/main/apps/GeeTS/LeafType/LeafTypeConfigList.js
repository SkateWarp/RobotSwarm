import { motion } from "framer-motion";
import { useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { Icon, IconButton, Typography } from "@mui/material";
import moment from "moment";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";
import DeleteRowElementDialog from "../../../../shared-components/DeleteRowElementDialog";
import useFilteredData from "../../../../shared-components/hooks/useFilteredData";
import GeneralTableFrontendPaginationComplete from "../../../../shared-components/GeneralTableFrontendPaginationComplete";
import { openEditLeafTypesConfigDialog, removeLeafType, selectLeafTypes } from "./store/leafTypeConfigSlice";
import TaskCategoriesLabel from "../../../../shared-components/TaskCategoriesLabel";

function LeafTypesConfigList() {
    const dispatch = useDispatch();
    const leafTypes = useSelector(selectLeafTypes);

    const { statusFilter, filteredData } = useFilteredData(leafTypes, "");
    const [openDialog, setOpenDialog] = useState(false);
    const [idForDelete, setIdForDelete] = useState(0);

    const handleClickOpen = () => {
        setOpenDialog(true);
    };

    const deleteRowElement = () => {
        dispatch(removeLeafType(idForDelete));
    };

    const columns = useMemo(
        () => [
            {
                Header: "Nombre",
                accessor: "name",
                sortable: true,
            },
            {
                Header: "Descripción",
                accessor: "taskTypeDescription",
                sortable: true,
            },
            {
                Header: "Fecha",
                accessor: "dateCreated",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        {moment(row.original.dateCreated).format("DD-MM-YYYY")}
                    </div>
                ),
            },
            {
                Header: "Categoría",
                className: "font-bold whitespace-no-wrap",
                width: 128,
                sortable: false,
                Cell: ({ row }) => {
                    return <TaskCategoriesLabel status={row.original.taskType} />;
                },
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

    if (statusFilter && !filteredData.length) {
        return (
            <div className="flex flex-1 items-center justify-center h-full">
                <Typography color="textSecondary" variant="h5">
                    No hay datos!
                </Typography>
            </div>
        );
    }
    if (!statusFilter) {
        return <ChargingProgressBar />;
    }

    return (
        <motion.div
            initial={{ y: 20, opacity: 0 }}
            animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
            className="flex flex-auto w-full max-h-full"
        >
            <GeneralTableFrontendPaginationComplete
                columns={columns}
                data={filteredData}
                onRowClick={(ev, row) => {
                    if (row) {
                        dispatch(openEditLeafTypesConfigDialog(row.original));
                    }
                }}
                rowsPerPageOptions={[5, 10, 15, 25]}
            />

            <DeleteRowElementDialog
                isOpen={openDialog}
                setIsOpen={setOpenDialog}
                deleteData={deleteRowElement}
                message="esta tarea"
            />
        </motion.div>
    );
}

export default LeafTypesConfigList;
