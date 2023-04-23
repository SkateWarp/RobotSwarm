import PropTypes from "prop-types";
import clsx from "clsx";
import { Typography } from "@mui/material";
import moment from "moment";
import { makeStyles } from "@mui/styles";

const useStyles = makeStyles((theme) => ({
    divider: {
        backgroundColor: theme.palette.divider,
    },
}));

// Todo cambiar fecha por hora de primera clasificación.
const PredictionsDetailsHeader = ({ productionDate, leafName, predictions, actualMachine }) => {
    const classes = useStyles();

    return (
        <div className="flex flex-col md:flex-row print:flex-row justify-between">
            <div className="flex flex-col">
                <Typography color="textSecondary" className="text-16">
                    <b>{actualMachine?.model}</b>
                </Typography>

                <Typography color="textSecondary" className="text-16">
                    <b>Operador:</b> {predictions[0].account?.firstName} {predictions[0].account?.lastName}
                </Typography>
            </div>

            <div
                className={clsx(
                    classes.divider,
                    "mx-32 md:w-px md:h-auto print:w-px print:h-auto print:mx-16"
                )}
            />
            <div className="mt-16 md:mt-0 print:mt-0">
                <Typography color="textSecondary" className="mb-8 font-light text-center" variant="h4">
                    {leafName}
                </Typography>
                <table className="w-full">
                    <tbody>
                        <tr>
                            <td className="text-center md:text-right print:text-right">
                                <Typography color="textSecondary">FECHA CREACIÓN:</Typography>
                            </td>
                            <td className="sm:px-16 print:px-16">
                                <Typography color="textSecondary">
                                    {moment(productionDate).format("DD-MM-YYYY h:mm A")}
                                </Typography>
                            </td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </div>
    );
};

PredictionsDetailsHeader.propTypes = {
    leafName: PropTypes.string.isRequired,
    predictions: PropTypes.array.isRequired,
    actualMachine: PropTypes.object,
    productionDate: PropTypes.string.isRequired,
};

export default PredictionsDetailsHeader;
