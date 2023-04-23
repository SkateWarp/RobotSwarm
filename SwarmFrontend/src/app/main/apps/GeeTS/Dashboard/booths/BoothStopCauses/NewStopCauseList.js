/* eslint-disable react-hooks/exhaustive-deps */
import { useCallback, useEffect, useMemo, useState } from "react";
import { Typography } from "@mui/material";
import { motion } from "framer-motion";
import { useDispatch, useSelector } from "react-redux";
import moment from "moment/moment";
import PropTypes from "prop-types";
import FragaStopTypeLabel from "../../../../../../shared-components/FragaStopTypeLabel";
import ChargingProgressBar from "../../../../../../shared-components/ChargingProgressBar";
import { getStopCause } from "../../store/boothDashboardSlice";
import GTSGeneralTable from "../../../GTSGeneralTable";

const NewStopCauseList = ({ stopCauses }) => {
    const dispatch = useDispatch();

    const paginationData = useSelector(
        ({ boothDashboardDetailsApp }) => boothDashboardDetailsApp.boothDashboards.pagination
    );

    const [filteredData, setFilteredData] = useState(null);
    const [statusFilter, setStatusFilter] = useState(false);

    const searchDataOnChangePage = (pageNumber) => {
        dispatch(getStopCause({ ...paginationData, pageNumber }));
    };

    const searchDataOnChangeRowsPerPage = (pageSize) => {
        dispatch(getStopCause({ ...paginationData, pageSize }));
    };

    const columns = useMemo(
        () => [
            {
                Header: "Código",
                accessor: "alarmReason.codigo",
                className: "",
                sortable: true,
            },
            {
                Header: "Tipo",
                className: "whitespace-no-wrap",
                accessor: "alarmType",
                width: 128,
                sortable: true,
                Cell: ({ row }) => {
                    return <FragaStopTypeLabel status={row.original.alarmReason?.stopType} />;
                },
            },
            {
                Header: "Descripción",
                accessor: "alarmReason.description",
                className: "",
                sortable: true,
            },
            {
                Header: "Operador",
                className: "",
                accessor: "operator",
                sortable: true,
                Cell: ({ row }) => (
                    <div className="flex items-center">
                        {row.original.operator?.firstName} {row.original.operator?.lastName}
                    </div>
                ),
            },
            {
                Header: "Fecha",
                width: 128,
                sortable: true,
                accessor: "date",
                Cell: ({ row }) => (
                    <div className="flex items-center">{moment(row.original.start).format("DD-MM-YYYY")}</div>
                ),
            },
            {
                Header: "Hora Inicio",
                width: 128,
                sortable: true,
                accessor: "startTime",
                Cell: ({ row }) => (
                    <div className="flex items-center">{moment(row.original.start).format("LT")}</div>
                ),
            },
            {
                Header: "Hora Fin",
                width: 128,
                sortable: true,
                accessor: "endTime",
                Cell: ({ row }) => (
                    <div className="flex items-center">{moment(row.original.end).format("LT")}</div>
                ),
            },
            {
                Header: "Duración",
                width: 128,
                sortable: false,
                accessor: "duration",
            },
        ],
        []
    );

    const handleSort = useCallback(
        (sort) => {
            if (sort.length > 0) {
                dispatch(
                    getStopCause({
                        ...paginationData,
                        pageNumber: 1,
                        sortColumn: sort[0].id,
                        sortDesc: sort[0].desc,
                    })
                );
            }
        },
        [stopCauses]
    );

    useEffect(() => {
        if (stopCauses) {
            setFilteredData(stopCauses);

            if (stopCauses.length) {
                setStatusFilter(true);
            } else if (filteredData !== null) {
                setStatusFilter(true);
            } else {
                setStatusFilter(false);
            }
        }
    }, [stopCauses]);

    if (statusFilter && filteredData.length === 0) {
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
        <motion.div initial={{ y: 20, opacity: 0 }} animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}>
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
};

NewStopCauseList.propTypes = {
    stopCauses: PropTypes.array.isRequired,
};

export default NewStopCauseList;
