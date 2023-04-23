/* eslint-disable react-hooks/exhaustive-deps */
import PropTypes from "prop-types";
import TablePagination from "@mui/material/TablePagination";
import { useGlobalFilter, usePagination, useRowSelect, useSortBy, useTable } from "react-table";
import { useEffect } from "react";
import { useParams } from "react-router-dom";
import { useDispatch } from "react-redux";
import GeneralTablePaginationActions from "../../../shared-components/GeneralTablePaginationActions";
import { getAccounts } from "./store/accountsSlice";
import GeneralTable from "../../../shared-components/GeneralTable";

const AccountsTable = ({ columns, data, pagination, onRowClick, onSort }) => {
    const dispatch = useDispatch();
    const routeParams = useParams();

    const {
        getTableProps,
        headerGroups,
        prepareRow,
        page,
        setPageSize,
        state: { pageSize, sortBy },
    } = useTable(
        {
            columns,
            data,
            autoResetPage: true,
            manualSortBy: true,
            disableMultiSort: true,
        },
        useGlobalFilter,
        useSortBy,
        usePagination,
        useRowSelect
    );

    useEffect(() => {
        onSort(sortBy);
        setPageSize(pagination.pageSize);
    }, [sortBy]);

    const handleChangePage = (event, newPage) => {
        event.stopPropagation();
        const pageNumber = newPage + 1;
        dispatch(
            getAccounts({
                ...routeParams,
                pageNumber,
                pageSize,
                searchFilter: pagination.searchFilter,
                sortColumn: pagination.sortColumn,
                sortDesc: pagination.sortDesc,
            })
        );
    };

    const handleChangeRowsPerPage = (event) => {
        event.stopPropagation();
        setPageSize(Number(event.target.value));

        dispatch(
            getAccounts({
                ...routeParams,
                pageSize: Number(event.target.value),
                searchFilter: pagination.searchFilter,
                sortColumn: pagination.sortColumn,
                sortDesc: pagination.sortDesc,
            })
        );
    };

    return (
        <div className="flex flex-col w-full min-h-full sm:border-1 sm:rounded-16 overflow-hidden">
            <GeneralTable
                getTableProps={getTableProps}
                headerGroups={headerGroups}
                page={page}
                prepareRow={prepareRow}
                onRowClick={onRowClick}
                pagination={pagination}
            />

            <TablePagination
                component="div"
                classes={{
                    root: "shrink-0 border-t-1",
                }}
                rowsPerPageOptions={[5, 10, 15, 25]}
                colSpan={5}
                count={pagination.totalRecords}
                rowsPerPage={pagination.pageSize}
                page={pagination.pageNumber - 1}
                SelectProps={{
                    inputProps: { "aria-label": "rows per page" },
                    native: false,
                }}
                onPageChange={handleChangePage}
                onRowsPerPageChange={handleChangeRowsPerPage}
                ActionsComponent={GeneralTablePaginationActions}
            />
        </div>
    );
};

AccountsTable.propTypes = {
    columns: PropTypes.array.isRequired,
    data: PropTypes.array.isRequired,
    onRowClick: PropTypes.func.isRequired,
    pagination: PropTypes.object.isRequired,
    onSort: PropTypes.func.isRequired,
};

export default AccountsTable;
