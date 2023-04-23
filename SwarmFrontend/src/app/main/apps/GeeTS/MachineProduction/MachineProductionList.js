/* eslint-disable react-hooks/exhaustive-deps */
import { motion } from "framer-motion";
import Typography from "@mui/material/Typography";
import moment from "moment";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useNavigate } from "react-router-dom";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";
import { getMachineProductions } from "./store/machineProductionSlice";
import GTSGeneralTable from "../GTSGeneralTable";

function MachineProductionList() {
    const navigate = useNavigate();
    const dispatch = useDispatch();

    const productions = useSelector(
        ({ machineProductionApp }) => machineProductionApp.machineProductions.productionSorting
    );

    const searchText = useSelector(
        ({ machineProductionApp }) => machineProductionApp.machineProductions.searchText
    );

    const paginationData = useSelector(
        ({ machineProductionApp }) => machineProductionApp.machineProductions.pagination
    );

    const [previousSearch, setPreviousSearch] = useState("");
    const [statusFilter, setStatusFilter] = useState(false);
    const [filteredData, setFilteredData] = useState(null);

    const searchDataOnChangePage = (pageNumber) => {
        dispatch(getMachineProductions({ ...paginationData, pageNumber }));
    };

    const searchDataOnChangeRowsPerPage = (pageSize) => {
        dispatch(getMachineProductions({ ...paginationData, pageSize }));
    };

    const columns = useMemo(
        () => [
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
                Header: "Máquina",
                accessor: "machine.model",
                sortable: true,
            },
            {
                Header: "Cantidad",
                accessor: "quantity",
                sortable: true,
            },
        ],
        []
    );

    useEffect(() => {
        if (productions.data) {
            if (previousSearch !== searchText && (searchText.length >= 3 || searchText === "")) {
                dispatch(
                    getMachineProductions({
                        searchFilter: searchText,
                        pageNumber: 1,
                        pageSize: paginationData.pageSize,
                        sortColumn: paginationData.sortColumn,
                        sortDesc: paginationData.sortDesc,
                    })
                );
                setPreviousSearch(searchText);
            }

            setFilteredData(productions.data);

            if (productions.data) {
                setStatusFilter(true);
            } else if (filteredData === null) {
                setStatusFilter(true);
            } else {
                setStatusFilter(false);
            }
        }
    }, [productions.data, searchText, previousSearch]);

    const handleSort = useCallback(
        (sort) => {
            if (sort.length > 0) {
                dispatch(
                    getMachineProductions({
                        ...paginationData,
                        pageNumber: 1,
                        searchFilter: searchText,
                        sortColumn: sort[0].id,
                        sortDesc: sort[0].desc,
                    })
                );
            }
        },
        [productions]
    );

    if (statusFilter && !filteredData.length) {
        return (
            <div className="flex flex-1 items-center justify-center h-full">
                <Typography color="textSecondary" variant="h5">
                    No hay reportes!
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
            <GTSGeneralTable
                onSort={handleSort}
                columns={columns}
                data={filteredData}
                pagination={paginationData}
                searchDataOnChangePage={searchDataOnChangePage}
                searchDataOnChangeRowsPerPage={searchDataOnChangeRowsPerPage}
                onRowClick={(ev, row) => {
                    if (row) {
                        navigate({
                            pathname: `/apps/GTS/production/machines/details/${row.original.dateCreated}/${row.original.machine.id}`,
                        });
                    }
                }}
            />
        </motion.div>
    );
}

export default MachineProductionList;
