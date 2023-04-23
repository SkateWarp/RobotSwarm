/* eslint-disable react-hooks/exhaustive-deps */
import { motion } from "framer-motion";
import Typography from "@mui/material/Typography";
import moment from "moment";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import ChargingProgressBar from "../../../../shared-components/ChargingProgressBar";
import { getOperatorProductions } from "./store/operatorProductionSlice";
import GTSGeneralTable from "../GTSGeneralTable";

function OperatorProductionList() {
    const dispatch = useDispatch();

    const productions = useSelector(
        ({ operatorProductionApp }) => operatorProductionApp.operatorProductions.productionSorting
    );

    const paginationData = useSelector(
        ({ operatorProductionApp }) => operatorProductionApp.operatorProductions.pagination
    );

    const [statusFilter, setStatusFilter] = useState(false);
    const [filteredData, setFilteredData] = useState(null);

    const searchDataOnChangePage = (pageNumber) => {
        dispatch(getOperatorProductions({ ...paginationData, pageNumber }));
    };

    const searchDataOnChangeRowsPerPage = (pageSize) => {
        dispatch(getOperatorProductions({ ...paginationData, pageSize }));
    };

    const columns = useMemo(
        () => [
            {
                Header: "Operador",
                className: "font-bold",
                accessor: "operator",
                sortable: true,
                Cell: ({ row }) => {
                    if (row.original.operator) {
                        return (
                            <Typography>
                                {row.original.operator.firstName} {row.original.operator.lastName}
                            </Typography>
                        );
                    }
                    return "";
                },
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
                Header: "Hojas Clasificadas",
                accessor: "leafCountQuantity.classifiedQuantity",
                sortable: true,
            },
            {
                Header: "Hojas No Clasificadas",
                accessor: "leafCountQuantity.unclassifiedQuantity",
                sortable: true,
            },
            {
                Header: "Total",
                accessor: "leafCountQuantity.totalQuantity",
                sortable: true,
            },
            {
                Header: "Takt Time (mm:ss)",
                accessor: "taktTime",
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
                    getOperatorProductions({
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
                onRowClick={() => {}}
            />
        </motion.div>
    );
}

export default OperatorProductionList;
