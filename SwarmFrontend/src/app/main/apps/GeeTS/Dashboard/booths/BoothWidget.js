/* eslint-disable react-hooks/exhaustive-deps */
import { useState, useEffect, memo } from "react";
import PropTypes from "prop-types";
import { Table, TableBody, TableCell, TableRow, Tooltip, Typography } from "@mui/material";
import BoothProduction from "./BoothProduction";

const textStyle = {
    height: "260px",
};

function BoothWidget({ robot }) {

    const [machineStopped, setMachineStopped] = useState(false);
    const [status, setStatus] = useState(true);
    const [bg, setBackground] = useState("");

    useEffect(() => {
        const bgOn = "#41CF3F";
        const bgOff = "#D9D9D9";
        const red = "#DC0000";

        const string = status ? bgOn : bgOff;
        setBackground(machineStopped ? red : string);
    }, [status, machineStopped]);

    return (
        <div className="w-full p-32">
            <div className="flex w-full justify-between">
                <div>
                    <Tooltip title="Encendida" arrow>
                        <div className="w-36 h-36">
                            <div
                                className="h2 p-8"
                                style={{
                                    backgroundColor: bg,
                                    display: "inline-flex",
                                    borderRadius: "25px",
                                    left: "80%",
                                }}
                            />
                        </div>
                    </Tooltip>
                </div>
            </div>
            <div className="flex flex-row flex-wrap items-end">
                <div className="flex flex-col m-auto">
                    <img
                        alt="logo"
                        src="assets/images/logos/booth.png"
                        className="inline-flex w-192 leading-none mx-auto"
                    />
                </div>
            </div>
            <div className="flex flex-row flex-wrap items-end">
                <div className="flex flex-col m-auto">
                    <Typography
                        className="inline-flex px-16 items-center py-auto text-32"
                        color="textSecondary"
                    >
                        test
                    </Typography>
                </div>
            </div>
            <Table className="mt-8 text-center bg-transparent">
                <TableBody>
                    <TableRow style={textStyle}>
                        <TableCell>
                            {/*<BoothProduction booth={booth} />*/}
                            test
                        </TableCell>
                    </TableRow>
                </TableBody>
            </Table>
        </div>
    );
}

BoothWidget.propTypes = {
    booth: PropTypes.object.isRequired,
};

export default memo(BoothWidget);
