import PropTypes from "prop-types";
import TablePagination from "@mui/material/TablePagination";
import { usePagination, useRowSelect, useSortBy, useTable } from "react-table";
import GeneralTablePaginationActions from "../../../shared-components/GeneralTablePaginationActions";
import GeneralTable from "../../../shared-components/GeneralTable";
import { ACCOUNTS_PAGE_SIZE } from "./accountsViewModel";

const AccountsTable = ({ columns, data, onRowClick }) => {
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
            initialState: {
                pageIndex: 0,
                pageSize: ACCOUNTS_PAGE_SIZE,
                sortBy: [{ id: "fullName", desc: false }],
            },
            disableMultiSort: true,
        },
        useSortBy,
        usePagination,
        useRowSelect
    );

    const pagination = {
        pageNumber: pageIndex + 1,
        pageSize,
        totalRecords: data.length,
    };

    return (
        <div className="flex flex-col w-full min-h-full border-1 rounded-8 overflow-hidden">
            <GeneralTable
                tableAriaLabel="Usuarios"
                getTableProps={getTableProps}
                headerGroups={headerGroups}
                page={page}
                prepareRow={prepareRow}
                onRowClick={onRowClick}
                getRowAriaLabel={(row) =>
                    `Editar usuario ${row.values.fullName || row.original.name || "sin nombre"}`
                }
                rowAriaHasPopup="dialog"
                pagination={pagination}
            />

            <TablePagination
                component="div"
                classes={{ root: "shrink-0 border-t-1" }}
                rowsPerPageOptions={[ACCOUNTS_PAGE_SIZE, 25, 50]}
                count={data.length}
                rowsPerPage={pageSize}
                page={pageIndex}
                labelRowsPerPage="Filas por página:"
                labelDisplayedRows={({ from, to, count }) => `${from}–${to} de ${count}`}
                SelectProps={{
                    inputProps: { "aria-label": "Filas por página" },
                    native: false,
                }}
                onPageChange={(event, newPage) => gotoPage(newPage)}
                onRowsPerPageChange={(event) => {
                    setPageSize(Number(event.target.value));
                    gotoPage(0);
                }}
                ActionsComponent={GeneralTablePaginationActions}
            />
        </div>
    );
};

AccountsTable.propTypes = {
    columns: PropTypes.array.isRequired,
    data: PropTypes.array.isRequired,
    onRowClick: PropTypes.func.isRequired,
};

export default AccountsTable;
