import { Typography } from "@mui/material";
import PropTypes from "prop-types";

function BoothProduction({ booth }) {
    return (
        <div className="flex flex-col">
            <div className="flex flex-col items-center">
                <Typography className="flex h2 text-center justify-items-center items-center self-center mt-16">
                    {booth.operator?.firstName} {booth.operator?.lastName}
                </Typography>
                <Typography className="h3" color="textSecondary">
                    Operador
                </Typography>
            </div>

            <div className="flex flex-col mt-16 items-center">
                <Typography className="h2 text-center">{booth.leafCountQuantity.totalQuantity}</Typography>

                <Typography className="h3" color="textSecondary">
                    Total Hojas
                </Typography>

                <Typography className="h2 text-center">
                    {booth.leafCountQuantity.classifiedQuantity}
                </Typography>

                <Typography className="h3" color="textSecondary">
                    Hojas Clasificadas
                </Typography>

                <Typography className="h2 text-center">
                    {booth.leafCountQuantity.unclassifiedQuantity}
                </Typography>

                <Typography className="h3" color="textSecondary">
                    Hojas No Clasificadas
                </Typography>

                <Typography className="mt-16 h2 text-center">{booth.taktTime}</Typography>

                <Typography className="h3" color="textSecondary">
                    Takt Time
                </Typography>
            </div>
        </div>
    );
}

BoothProduction.propTypes = {
    booth: PropTypes.shape({
        dateCreated: PropTypes.string.isRequired,
        taktTime: PropTypes.string.isRequired,

        leafCountQuantity: PropTypes.shape({
            totalQuantity: PropTypes.number.isRequired,
            classifiedQuantity: PropTypes.number.isRequired,
            unclassifiedQuantity: PropTypes.number.isRequired,
        }),

        machine: PropTypes.shape({
            id: PropTypes.number.isRequired,
            model: PropTypes.string.isRequired,
        }),
    }),
};

export default BoothProduction;
