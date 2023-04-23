import PropTypes from "prop-types";
import { Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Typography } from "@mui/material";
import Paper from "@mui/material/Paper/Paper";
import Button from "@mui/material/Button";

// Esta tabla se encargará de mostrar la información de cada uno de los tipos de hojas realizadas en la producción
// En otras palabras en esta tabla estarán las informaciones agrupadas por los distintos tipos de hojas.
const TobaccoProductionDetailsTable = ({ totalTobaccoProduction }) => {
    return (
        <div className="print:mt-0">
            <Typography color="textSecondary" className="text-16">
                <b>Tipo de Hoja:</b> {totalTobaccoProduction.leafType.name}
            </Typography>

            <Typography color="textSecondary" className="text-16 mb-16">
                <b>Cantidad de hojas:</b> {totalTobaccoProduction.totalQuantity}
            </Typography>

            <TableContainer component={Paper} className="text-16 mb-36">
                <Table aria-label="simple table" key={totalTobaccoProduction.id}>
                    <TableHead>
                        <TableRow>
                            <TableCell align="center">#</TableCell>
                            <TableCell />
                            <TableCell align="center">Cantidad</TableCell>
                            <TableCell align="center">Operador</TableCell>
                            <TableCell align="center" className="print:hidden" />
                        </TableRow>
                    </TableHead>
                    <TableBody>
                        {totalTobaccoProduction.sortingProductions.map((tobaccoProduction, index) => (
                            <TableRow key={index}>
                                <TableCell align="center">{index + 1}</TableCell>
                                <TableCell align="center" />
                                <TableCell align="center">{tobaccoProduction.quantity}</TableCell>
                                <TableCell align="center">
                                    {tobaccoProduction.operator.firstName}{" "}
                                    {tobaccoProduction.operator.lastName}
                                </TableCell>
                                <TableCell align="center" className="print:hidden">
                                    <Button
                                        variant="contained"
                                        onClick={() =>
                                            window.open(
                                                `/apps/GTS/predictions/details/${tobaccoProduction.dateCreated}/${tobaccoProduction.operator.id}/${totalTobaccoProduction.leafType.name}`,
                                                "_blank"
                                            )
                                        }
                                    >
                                        Predicciones
                                    </Button>
                                </TableCell>
                            </TableRow>
                        ))}
                    </TableBody>
                </Table>
            </TableContainer>
        </div>
    );
};

TobaccoProductionDetailsTable.propTypes = {
    totalTobaccoProduction: PropTypes.object.isRequired,
};

export default TobaccoProductionDetailsTable;
