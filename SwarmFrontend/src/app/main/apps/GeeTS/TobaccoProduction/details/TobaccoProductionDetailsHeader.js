import PropTypes from "prop-types";
import clsx from "clsx";
import { Typography } from "@mui/material";
import moment from "moment";
import { LOGO } from "app/constants/constants";
import { makeStyles } from "@mui/styles";

const useStyles = makeStyles((theme) => ({
    divider: {
        backgroundColor: theme.palette.divider,
    },
}));

const TobaccoProductionDetailsHeader = ({ productionDate, headerName }) => {
    const classes = useStyles();

    return (
        <div className="flex flex-col md:flex-row print:flex-row justify-between">
            <div className="flex flex-col justify-center">
                <div className="flex items-center justify-center print:mb-0">
                    <img className="w-180 print:w-160" src={LOGO} alt="logo" />
                </div>
            </div>
            <div
                className={clsx(
                    classes.divider,
                    "mx-32 md:w-px md:h-auto print:w-px print:h-auto print:mx-16"
                )}
            />
            <div className="mt-16 md:mt-0 print:mt-0">
                <Typography color="textSecondary" className="mb-8 font-light text-center" variant="h4">
                    {headerName}
                </Typography>
                <table className="w-full">
                    <tbody>
                        <tr>
                            <td className="text-center md:text-right print:text-right">
                                <Typography color="textSecondary">FECHA CREACIÓN:</Typography>
                            </td>
                            <td className="sm:px-16 print:px-16">
                                <Typography color="textSecondary">
                                    {moment(productionDate).format("DD-MM-YYYY")}
                                </Typography>
                            </td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </div>
    );
};

TobaccoProductionDetailsHeader.propTypes = {
    productionDate: PropTypes.string.isRequired,
    headerName: PropTypes.string.isRequired,
};

export default TobaccoProductionDetailsHeader;
