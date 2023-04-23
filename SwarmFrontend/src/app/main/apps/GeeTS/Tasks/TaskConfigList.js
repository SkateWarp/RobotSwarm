import { motion } from "framer-motion";
import Icon from "@mui/material/Icon";
import IconButton from "@mui/material/IconButton";
import Typography from "@mui/material/Typography";
import { useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { openEditTaskConfigDialog, removeTaskType, selectTaskType } from "./store/taskConfigSlice";
import DeleteRowElementDialog from "../../../../shared-components/DeleteRowElementDialog";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";
import TaskCategoriesLabel from "../../../../shared-components/TaskCategoriesLabel";
import useFilteredData from "../../../../shared-components/hooks/useFilteredData";
import GeneralTableFrontendPaginationComplete from "../../../../shared-components/GeneralTableFrontendPaginationComplete";

function TaskConfigList() {
    const dispatch = useDispatch();
    const taskTypes = useSelector(selectTaskType);
    const searchText = useSelector(({ taskConfigApp }) => taskConfigApp.taskConfig.searchText);

    const { statusFilter, filteredData } = useFilteredData(taskTypes, searchText);
    const [openDialog, setOpenDialog] = useState(false);
    const [idForDelete, setIdForDelete] = useState(0);

    const handleClickOpen = () => {
        setOpenDialog(true);
    };

    const deleteTask = () => {
        dispatch(removeTaskType(idForDelete));
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
                accessor: "description",
                sortable: true,
            },

            {
                Header: "Categoría",
                className: "font-bold whitespace-no-wrap",
                width: 128,
                sortable: false,
                Cell: ({ row }) => {
                    return <TaskCategoriesLabel status={row.original.taskCategory.id} />;
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
        <>
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
                            dispatch(openEditTaskConfigDialog(row.original));
                        }
                    }}
                    rowsPerPageOptions={[5, 10, 15, 25]}
                />
            </motion.div>

            <DeleteRowElementDialog
                isOpen={openDialog}
                setIsOpen={setOpenDialog}
                deleteData={deleteTask}
                message="este sensor"
            />
        </>
    );
}

export default TaskConfigList;
