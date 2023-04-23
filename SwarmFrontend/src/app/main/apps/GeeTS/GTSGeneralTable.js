import PropTypes from "prop-types";
import TablePagination from "@mui/material/TablePagination";
import { useGlobalFilter, usePagination, useRowSelect, useSortBy, useTable } from "react-table";
import { useEffect } from "react";
import GeneralTablePaginationActions from "../../../shared-components/GeneralTablePaginationActions";
import GeneralTable from "../../../shared-components/GeneralTable";

const GTSGeneralTable = ({
    columns,
    data,
    pagination,
    onRowClick,
    onSort,
    searchDataOnChangePage,
    searchDataOnChangeRowsPerPage,
}) => {
    const {
        getTableProps,
        headerGroups,
        prepareRow,
        page,
        setPageSize,
        state: { sortBy },
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

        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [sortBy]);

    const handleChangePage = (event, newPage) => {
        event.stopPropagation();
        const pageNumber = newPage + 1;
        searchDataOnChangePage(pageNumber);
    };

    const handleChangeRowsPerPage = (event) => {
        event.stopPropagation();

        const actualPageSize = Number(event.target.value);
        setPageSize(actualPageSize);
        searchDataOnChangeRowsPerPage(actualPageSize);
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
                rowsPerPageOptions={[10, 25, 50, 100]}
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

GTSGeneralTable.propTypes = {
    data: PropTypes.array.isRequired,
    onSort: PropTypes.func.isRequired,
    columns: PropTypes.array.isRequired,
    onRowClick: PropTypes.func.isRequired,
    pagination: PropTypes.object.isRequired,
    searchDataOnChangePage: PropTypes.func.isRequired,
    searchDataOnChangeRowsPerPage: PropTypes.func.isRequired,
};

export default GTSGeneralTable;
