/* eslint-disable react-hooks/exhaustive-deps */
import { motion } from "framer-motion";
import Typography from "@mui/material/Typography";
import moment from "moment";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useNavigate } from "react-router-dom";
import { getTobaccoProductions } from "./store/tobaccoProductionSlice";
import GTSGeneralTable from "../GTSGeneralTable";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";

function TobaccoProductionList() {
    const navigate = useNavigate();
    const dispatch = useDispatch();

    const productions = useSelector(
        ({ tobaccoProductionApp }) => tobaccoProductionApp.tobaccoProductions.productionSorting
    );

    const paginationData = useSelector(
        ({ tobaccoProductionApp }) => tobaccoProductionApp.tobaccoProductions.pagination
    );

    const [statusFilter, setStatusFilter] = useState(false);
    const [filteredData, setFilteredData] = useState(null);

    const searchDataOnChangePage = (pageNumber) => {
        dispatch(getTobaccoProductions({ ...paginationData, pageNumber }));
    };

    const searchDataOnChangeRowsPerPage = (pageSize) => {
        dispatch(getTobaccoProductions({ ...paginationData, pageSize }));
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
                Header: "Turno",
                accessor: "shift.description",
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
            setFilteredData(productions.data);

            if (productions.data.length) {
                setStatusFilter(true);
            } else if (filteredData === null) {
                setStatusFilter(true);
            } else {
                setStatusFilter(false);
            }
        }
    }, [productions.data]);

    const handleSort = useCallback(
        (sort) => {
            if (sort.length > 0) {
                dispatch(
                    getTobaccoProductions({
                        ...paginationData,
                        pageNumber: 1,
                        sortColumn: sort[0].id,
                        sortDesc: sort[0].desc,
                    })
                );
            }
        },
        [productions.data]
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
                            pathname: `/apps/GTS/production/leafs/details/${row.original.dateCreated}/${row.original.shift.id}`,
                        });
                    }
                }}
            />
        </motion.div>
    );
}

export default TobaccoProductionList;
