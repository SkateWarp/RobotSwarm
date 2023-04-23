/* eslint-disable react-hooks/exhaustive-deps */
import { useEffect, useState } from "react";
import { motion } from "framer-motion";
import axios from "axios";
import { useParams } from "react-router-dom";
import moment from "moment/moment";
import Typography from "@mui/material/Typography";
import { Card, Table, TableBody, TableCell, TableContainer, TableHead, TableRow } from "@mui/material";
import AccessTimeIcon from "@mui/icons-material/AccessTime";
import FlagIcon from "@mui/icons-material/Flag";
import ErrorOutlineIcon from "@mui/icons-material/ErrorOutline";
import EmojiFlagsIcon from "@mui/icons-material/EmojiFlags";
import GradingIcon from "@mui/icons-material/Grading";
import AccessAlarmIcon from "@mui/icons-material/AccessAlarm";
import AlarmOffIcon from "@mui/icons-material/AlarmOff";
import { URL } from "../../../../../constants/constants";
import ChargingProgressBar from "../../../../../shared-components/ChargingProgressBar";
import BackButton from "../../../../../fuse-layouts/shared-components/BackButton";

const bandStylesTitle = {
    fontSize: "16px",
};

const bandStylesBody = {
    fontSize: "20px",
};

const currentProductionInitialState = {
    leafCountQuantity: { totalQuantity: 0, classifiedQuantity: 0, unclassifiedQuantity: 0 },
    totalSortingProductions: [{ totalQuantity: 0, leafType: { name: "", description: "" } }],
};

const currentShiftInitialState = {
    description: "",
    starTime: "",
    endTime: "",
};

function ShiftsDashboardDetails() {
    const { date, shiftId } = useParams();

    const [hasDataFinishLoading, setHasDataFinishLoading] = useState(false);
    const [currentShiftData, setCurrentShiftData] = useState(currentShiftInitialState);
    const [currentProductionData, setCurrentProductionData] = useState(currentProductionInitialState);

    useEffect(() => {
        axios.get(`${URL}/api/SortingProduction/shift/details/${date}/${shiftId}`).then((res) => {
            const filteredData = res.data.totalSortingProductions.filter(
                (production) => production.leafType.tag !== "undetermined"
            );

            setCurrentProductionData({
                leafCountQuantity: res.data.leafCountQuantity,
                totalSortingProductions: filteredData,
            });

            setHasDataFinishLoading(true);
        });

        axios.get(`${URL}/api/Shift/${shiftId}`).then((res) => {
            setCurrentShiftData(res.data);
        });
    }, []);

    return hasDataFinishLoading ? (
        <>
            <BackButton className="w-32 ml-16" />

            <Typography className="text-center font-bebasNeue pt-16" color="textSecondary" variant="h3">
                {currentShiftData.description}
            </Typography>
            <motion.div
                enter={{
                    animation: "transition.slideUpIn",
                }}
                delay={200}
            >
                <div className="flex flex-1 flex-col min-w-0">
                    <div className="flex flex-col flex-wrap px-32 md:px-24">
                        <div className="flex flex-col sm:flex-row md:justify-center">
                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <AccessTimeIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography
                                        className="justify-items-center"
                                        style={bandStylesTitle}
                                        color="textSecondary"
                                    >
                                        Fecha
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {moment(date).format("YYYY-MM-DD")}
                                    </Typography>
                                </div>
                            </div>
                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <FlagIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography style={bandStylesTitle} color="textSecondary">
                                        Total de Hojas
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {currentProductionData.leafCountQuantity.totalQuantity}
                                    </Typography>
                                </div>
                            </div>
                        </div>

                        <div className="flex flex-col sm:flex-row md:justify-center">
                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <ErrorOutlineIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography
                                        className="justify-items-center"
                                        style={bandStylesTitle}
                                        color="textSecondary"
                                    >
                                        Total de Hojas no clasificadas
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {currentProductionData.leafCountQuantity.unclassifiedQuantity}
                                    </Typography>
                                </div>
                            </div>

                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <EmojiFlagsIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography
                                        className="justify-items-center"
                                        style={bandStylesTitle}
                                        color="textSecondary"
                                    >
                                        Total de Hojas clasificadas
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {currentProductionData.leafCountQuantity.classifiedQuantity}
                                    </Typography>
                                </div>
                            </div>
                        </div>
                        <div className="flex flex-col sm:flex-row md:justify-center">
                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <AccessAlarmIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography
                                        className="justify-items-center"
                                        style={bandStylesTitle}
                                        color="textSecondary"
                                    >
                                        Inicio de Turno
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {currentShiftData.startTime}
                                    </Typography>
                                </div>
                            </div>

                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <AlarmOffIcon color="action" fontSize="large" />

                                <div className="mt-4 ml-8">
                                    <Typography
                                        className="justify-items-center"
                                        style={bandStylesTitle}
                                        color="textSecondary"
                                    >
                                        Fin de Turno
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {currentShiftData.endTime}
                                    </Typography>
                                </div>
                            </div>
                        </div>
                    </div>

                    <div className="flex flex-col sm:flex-row align-center ml-36 pt-12 mt-32">
                        <GradingIcon color="action" fontSize="large" />

                        <Typography className="sm:pr-16 text-18 h2 font-bold">
                            Tipos de Hojas Clasificadas
                        </Typography>
                    </div>

                    <div className="widget flex p-16 mx-8 md:mx-24">
                        {currentProductionData.totalSortingProductions.length > 0 ? (
                            <Card className="w-full rounded-8 shadow-1">
                                <Typography className="text-center text-16 font-400" />
                                <TableContainer>
                                    <Table size="small" aria-label="a dense table ">
                                        <TableHead>
                                            <TableRow>
                                                <TableCell>Tipo de Hoja</TableCell>
                                                <TableCell>Cantidad</TableCell>
                                            </TableRow>
                                        </TableHead>
                                        <TableBody>
                                            {currentProductionData.totalSortingProductions.map(
                                                (shiftsProduction, index) => (
                                                    <TableRow key={index}>
                                                        <TableCell component="th" scope="row">
                                                            {shiftsProduction.leafType.description}
                                                        </TableCell>
                                                        <TableCell component="th" scope="row">
                                                            {shiftsProduction.totalQuantity}
                                                        </TableCell>
                                                    </TableRow>
                                                )
                                            )}
                                        </TableBody>
                                    </Table>
                                </TableContainer>
                            </Card>
                        ) : (
                            <Typography color="textSecondary" variant="h6" className="m-32">
                                No hay información
                            </Typography>
                        )}
                    </div>
                </div>
            </motion.div>
        </>
    ) : (
        <ChargingProgressBar />
    );
}

export default ShiftsDashboardDetails;
