import PropTypes from "prop-types";
import { Button, TableBody, TableCell, TableRow } from "@mui/material";
import PhotoIcon from "@mui/icons-material/Photo";

const colorChart = {
    red: "#DE2D2D",
    yellow: "#d49503",
    green: "#28BF26",
};

const PredictionsDetailsTableBody = ({ prediction, predictedLeafName }) => {
    const handlePercentageColor = (percentageValue, actualLeafName) => {
        if (predictedLeafName === actualLeafName && percentageValue < 80.0) {
            return colorChart.yellow;
        }

        if (predictedLeafName !== actualLeafName && percentageValue > 80.0) {
            return colorChart.red;
        }

        return "";
    };

    const goToLeafPicture = () => {
        // window.open(`/apps/GTS/pics/${prediction.fileName}`, "_blank");
    };

    return (
        <TableBody>
            <TableRow>
                <TableCell className="print:hidden" align="center">
                    <Button
                        className="ml-6"
                        variant="contained"
                        onClick={goToLeafPicture}
                        endIcon={<PhotoIcon />}
                    >
                        Ver
                    </Button>
                </TableCell>
                {prediction.leafPredictions.map((leafPrediction) => (
                    <TableCell
                        key={leafPrediction.name}
                        align="center"
                        style={{
                            color: handlePercentageColor(100 * leafPrediction.value, leafPrediction.name),
                        }}
                    >
                        {(100 * leafPrediction.value).toFixed(2)}%
                    </TableCell>
                ))}
            </TableRow>
        </TableBody>
    );
};

PredictionsDetailsTableBody.propTypes = {
    prediction: PropTypes.object.isRequired,
    predictedLeafName: PropTypes.string.isRequired,
};

export default PredictionsDetailsTableBody;
