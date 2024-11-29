import {memo} from "react";
import PropTypes from "prop-types";
import {Table, TableBody, TableCell, TableRow, Tooltip, Typography} from "@mui/material";
import {LOGO} from "../../../../../constants/constants";

function RobotWidget({robot}) {

    return (

        <div className="w-full p-32">
            <div className="flex w-full justify-between">
                <div>
                    <Tooltip title="Encendida" arrow>
                        <div className="w-36 h-36">
                            <div
                                className="h2 p-8"
                                style={{
                                    backgroundColor: robot.isConnected ? "green" : "red",
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
                        src={LOGO}
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
                        {robot?.name}
                    </Typography>
                </div>
            </div>
            <Table className="mt-8 text-center bg-transparent">
                <TableBody>
                    <TableRow style={{
                        height: "260px",
                    }}>
                        <TableCell>

                            <div className="flex flex-col">
                                <div className="flex flex-col items-center">
                                    <Typography
                                        className="flex h2 text-center justify-items-center items-center self-center mt-16">
                                        {robot?.description}
                                    </Typography>
                                    <Typography className="h3" color="textSecondary">
                                        Descripcion
                                    </Typography>
                                </div>

                                <div className="flex flex-col items-center">
                                    <Typography
                                        className="flex h2 text-center justify-items-center items-center self-center mt-16">
                                        {robot?.statusDescription}
                                    </Typography>
                                    <Typography className="h3" color="textSecondary">
                                        Status
                                    </Typography>
                                </div>
                            </div>

                        </TableCell>
                    </TableRow>
                </TableBody>
            </Table>
        </div>
    );
}

RobotWidget.propTypes = {
    robot: PropTypes.object.isRequired,
};

export default memo(RobotWidget);
