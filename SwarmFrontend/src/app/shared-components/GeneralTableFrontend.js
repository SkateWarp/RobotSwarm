import PropTypes from "prop-types";
import {
    Table,
    TableBody,
    TableCell,
    TableContainer,
    TableHead,
    TableRow,
    TableSortLabel,
} from "@mui/material";
import clsx from "clsx";

const GeneralTableFrontend = ({ getTableProps, headerGroups, page, prepareRow, onRowClick, emptyRows }) => {
    // pongo esta condicional para evitar que la app no de error cuando no se manden datos en la tabla
    const cellsCount = Object.keys(page[0] ? page[0].values : []).length;

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
                                            // react-table has an unsorted state which is not treated here
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

GeneralTableFrontend.propTypes = {
    getTableProps: PropTypes.func.isRequired,
    headerGroups: PropTypes.array.isRequired,
    page: PropTypes.array.isRequired,
    prepareRow: PropTypes.func.isRequired,
    emptyRows: PropTypes.number.isRequired,
    onRowClick: PropTypes.func,
};

export default GeneralTableFrontend;
