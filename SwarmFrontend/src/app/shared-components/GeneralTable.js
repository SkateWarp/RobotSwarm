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

export const activateTableRowFromKeyboard = (event, onRowClick, row) => {
    if (event.target !== event.currentTarget || !["Enter", " "].includes(event.key)) return;
    event.preventDefault();
    onRowClick(event, row);
};

// Componente general para ser utilizado en todas las tablas
const GeneralTable = ({
    getTableProps,
    headerGroups,
    page,
    prepareRow,
    onRowClick,
    pagination,
    tableAriaLabel,
    getRowAriaLabel,
    rowAriaHasPopup,
}) => {
    const cellsCount = Object.keys(page[0]?.values ?? []).length;
    const emptyRows =
        pagination.pageNumber > 0
            ? Math.max(0, pagination.pageNumber * pagination.pageSize - pagination.totalRecords)
            : 0;
    const rowsAreInteractive = typeof onRowClick === "function";

    return (
        <TableContainer className="flex flex-1">
            <Table
                {...getTableProps()}
                aria-label={tableAriaLabel}
                stickyHeader
                size="small"
                className="simple"
            >
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
                                tabIndex={rowsAreInteractive ? 0 : undefined}
                                aria-label={
                                    rowsAreInteractive && getRowAriaLabel
                                        ? getRowAriaLabel(row)
                                        : undefined
                                }
                                aria-haspopup={rowsAreInteractive ? rowAriaHasPopup : undefined}
                                onClick={rowsAreInteractive ? (ev) => onRowClick(ev, row) : undefined}
                                onKeyDown={
                                    rowsAreInteractive
                                        ? (event) => activateTableRowFromKeyboard(event, onRowClick, row)
                                        : undefined
                                }
                                className={clsx("truncate", rowsAreInteractive && "cursor-pointer")}
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
    tableAriaLabel: PropTypes.string,
    getRowAriaLabel: PropTypes.func,
    rowAriaHasPopup: PropTypes.oneOf(["dialog", "menu", "listbox", "tree", "grid"]),
};

GeneralTable.defaultProps = {
    onRowClick: undefined,
    tableAriaLabel: undefined,
    getRowAriaLabel: undefined,
    rowAriaHasPopup: undefined,
};

export default GeneralTable;
