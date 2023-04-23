import { LinearProgress, Typography } from "@mui/material";

const ChargingProgressBar = () => {
    return (
        <div className="flex flex-1 flex-col items-center justify-center p-24">
            <Typography className="text-13 sm:text-20 mb-16" color="textSecondary">
                Cargando...
            </Typography>
            <LinearProgress className="w-192 sm:w-320 max-w-full rounded-2" color="secondary" />
        </div>
    );
};

export default ChargingProgressBar;
