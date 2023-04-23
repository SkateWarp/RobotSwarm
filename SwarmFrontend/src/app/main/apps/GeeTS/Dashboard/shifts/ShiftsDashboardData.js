import Typography from "@mui/material/Typography";
import PropTypes from "prop-types";

function ShiftsDashboardData({ production }) {
    return (
        <div className="flex flex-col sm:flex-row">
            <div className="flex w-full sm:w-1/2 justify-center md:w-1/3 mb-16 md:m-auto">
                <div className="mt-4 ml-8">
                    <Typography color="textSecondary" className="text-2xl text-left md:text-center">
                        Turno
                    </Typography>
                    <Typography
                        className="font-bold text-left md:text-center"
                        color="textPrimary"
                        variant="h6"
                    >
                        {production.shift?.description}
                    </Typography>
                </div>
            </div>
            <div className="flex w-full justify-center sm:w-1/2 md:w-1/4 mb-16 md:m-auto">
                <div className="mt-4 ml-8">
                    <Typography color="textSecondary" className="text-2xl text-left md:text-center">
                        Total
                    </Typography>
                    <Typography
                        className="font-bold text-left md:text-center"
                        variant="h6"
                        color="textPrimary"
                    >
                        {production.leafCountQuantity?.totalQuantity}
                    </Typography>
                </div>
            </div>
        </div>
    );
}

ShiftsDashboardData.propTypes = {
    production: PropTypes.object.isRequired,
};

export default ShiftsDashboardData;
