import PropTypes from "prop-types";
import { useGlobalFilter, usePagination, useRowSelect, useSortBy, useTable } from "react-table";
import TablePagination from "@mui/material/TablePagination";
import GeneralTablePaginationActions from "./GeneralTablePaginationActions";
import GeneralTableFrontend from "./GeneralTableFrontend";

const GeneralTableFrontendPaginationComplete = ({ columns, data, onRowClick, rowsPerPageOptions }) => {
    const {
        getTableProps,
        headerGroups,
        prepareRow,
        page,
        gotoPage,
        setPageSize,
        state: { pageIndex, pageSize },
    } = useTable(
        {
            columns,
            data,
            autoResetPage: true,
        },
        useGlobalFilter,
        useSortBy,
        usePagination,
        useRowSelect
    );

    const handleChangePage = (event, newPage) => {
        gotoPage(newPage);
    };

    const handleChangeRowsPerPage = (event) => {
        setPageSize(Number(event.target.value));
    };

    const emptyRows = pageIndex + 1 > 0 ? Math.max(0, (pageIndex + 1) * pageSize - data.length) : 0;

    return (
        <div className="flex flex-col w-full sm:border-1 sm:rounded-16 overflow-hidden">
            <GeneralTableFrontend
                getTableProps={getTableProps}
                headerGroups={headerGroups}
                page={page}
                prepareRow={prepareRow}
                emptyRows={emptyRows}
                onRowClick={onRowClick}
            />

            <TablePagination
                component="div"
                classes={{
                    root: "shrink-0 border-t-1",
                }}
                rowsPerPageOptions={rowsPerPageOptions}
                colSpan={5}
                count={data.length}
                rowsPerPage={pageSize}
                page={pageIndex}
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

GeneralTableFrontendPaginationComplete.propTypes = {
    columns: PropTypes.array.isRequired,
    data: PropTypes.array.isRequired,
    onRowClick: PropTypes.func.isRequired,
    rowsPerPageOptions: PropTypes.array.isRequired,
};

export default GeneralTableFrontendPaginationComplete;
