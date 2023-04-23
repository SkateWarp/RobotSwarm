import PropTypes from "prop-types";
import { Button } from "@mui/material";
import { useEffect } from "react";
import axios from "axios";
import { URL } from "../../../../constants/constants";

const test = [];

// Posible componente de paginación para las cabinas. Por ahora lo mantendré aqui.
const CustomPagination = ({ setBooths, booths }) => {
    useEffect(() => {
        const defaultValue = goToSelectedItems(test, [1, 4]);

        axios.get(`${URL}/api/SortingProduction/machines`).then((response) => {
            setBooths(response.data);
        });

        if (booths.length < 1) setBooths(defaultValue);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const goToSelectedItems = (data, index) => {
        const dataShowed = [];

        // eslint-disable-next-line no-plusplus
        for (let i = index[0]; i <= index[1]; i++) {
            data.forEach((value) => {
                if (value.id === i) {
                    dataShowed.push(value);
                }
            });
        }

        return dataShowed;
    };

    const handleClick = (index) => {
        const dataIndex = goToSelectedItems(test, index);
        setBooths(dataIndex);
    };

    return (
        <div>
            <div className="z-20 flex fixed top-160 left-136">
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([1, 4])}
                >
                    01-04
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([5, 8])}
                >
                    05-08
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([9, 12])}
                >
                    09-12
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([13, 16])}
                >
                    13-16
                </Button>
            </div>
            <div className="z-20 flex fixed top-160 left-136 mt-36">
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([17, 20])}
                >
                    17-20
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([21, 24])}
                >
                    21-24
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([25, 28])}
                >
                    25-28
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([29, 32])}
                >
                    29-32
                </Button>
            </div>
            <div className="z-20 flex fixed top-160 left-136 mt-72">
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([33, 36])}
                >
                    33-36
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([37, 40])}
                >
                    37-40
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([41, 44])}
                >
                    41-44
                </Button>
                <Button
                    sx={{
                        borderRadius: 1,
                        border: "1px solid",
                        borderColor: "gray",
                        "& .MuiButton-startIcon": { margin: 0 },
                    }}
                    variant="outlined"
                    onClick={() => handleClick([45, 48])}
                >
                    45-48
                </Button>
            </div>
        </div>
    );
};

CustomPagination.propTypes = {
    booths: PropTypes.array.isRequired,
    setBooths: PropTypes.func.isRequired,
};

export default CustomPagination;
