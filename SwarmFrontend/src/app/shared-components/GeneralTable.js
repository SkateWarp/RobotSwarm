import PropTypes from "prop-types";
import clsx from "clsx";
import {
    Table,
    TableBody,
    TableCell,
    TableContainer,
    TableHead,
    TableRow,
    TableSortLabel,
} from "@mui/material";

// Componente general para ser utilizado en todas las tablas
const GeneralTable = ({ getTableProps, headerGroups, page, prepareRow, onRowClick, pagination }) => {
    const cellsCount = Object.keys(page[0]?.values ?? []).length;
    const emptyRows =
        pagination.pageNumber > 0
            ? Math.max(0, pagination.pageNumber * pagination.pageSize - pagination.totalRecords)
            : 0;

    return (
        <TableContainer className="flex flex-1">
            <Table {...getTableProps()} stickyHeader size="small" className="simple">
                <TableHead>
                    {headerGroups.map((headerGroup) => (
                        <TableRow {...headerGroup.getHeaderGroupProps()}>
                            {headerGroup.headers.map((column) => (
                                <TableCell
                                    className="whitespace-no-wrap p-4 md:p-4"
                                    {...(!column.sortable
                                        ? column.getHeaderProps()
                                        : column.getHeaderProps(column.getSortByToggleProps()))}
                                >
                                    {column.render("Header")}
                                    {column.sortable ? (
                                        <TableSortLabel
                                            active={column.isSorted}
                                            direction={column.isSortedDesc ? "desc" : "asc"}
                                        />
                                    ) : null}
                                </TableCell>
                            ))}
                        </TableRow>
                    ))}
                </TableHead>
                <TableBody>
                    {page.map((row) => {
                        prepareRow(row);
                        return (
                            <TableRow
                                {...row.getRowProps()}
                                onClick={(ev) => onRowClick(ev, row)}
                                className="truncate cursor-pointer"
                                style={{
                                    height: 47,
                                    minHeight: 47,
                                }}
                            >
                                {row.cells.map((cell) => {
                                    return (
                                        <TableCell
                                            {...cell.getCellProps()}
                                            className={clsx("p-4 md:p-4", cell.column.className)}
                                        >
                                            {cell.render("Cell")}
                                        </TableCell>
                                    );
                                })}
                            </TableRow>
                        );
                    })}
                    {emptyRows > 0 && (
                        <TableRow
                            style={{
                                height: 47 * emptyRows,
                            }}
                        >
                            <TableCell colSpan={cellsCount} />
                        </TableRow>
                    )}
                </TableBody>
            </Table>
        </TableContainer>
    );
};

GeneralTable.propTypes = {
    getTableProps: PropTypes.func.isRequired,
    headerGroups: PropTypes.array.isRequired,
    page: PropTypes.array.isRequired,
    prepareRow: PropTypes.func.isRequired,
    onRowClick: PropTypes.func,
    pagination: PropTypes.object.isRequired,
};

export default GeneralTable;
