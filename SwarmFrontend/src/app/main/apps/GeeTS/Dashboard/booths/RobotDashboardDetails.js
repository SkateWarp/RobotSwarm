/* eslint-disable react-hooks/exhaustive-deps */
import { useEffect, useState } from "react";
import { Typography } from "@mui/material";
import { motion } from "framer-motion";
import axios from "axios";
import { useParams } from "react-router-dom";
import AccessTimeIcon from "@mui/icons-material/AccessTime";
import AccessAlarmIcon from "@mui/icons-material/AccessAlarm";
import AlarmOffIcon from "@mui/icons-material/AlarmOff";
import FlagIcon from "@mui/icons-material/Flag";
import { URL } from "../../../../../constants/constants";
import withReducer from "../../../../../store/withReducer";
import reducer from "../store";
import ChargingProgressBar from "../../../../../shared-components/ChargingProgressBar";
import BackButton from "../../../../../fuse-layouts/shared-components/BackButton";
import jwtService from "../../../../../services/jwtService";

const bandStylesTitle = {
    fontSize: "16px",
};

const bandStylesBody = {
    fontSize: "20px",
};

const currentProductionInitialState = {
    leafCountQuantity: { totalQuantity: 0, classifiedQuantity: 0, unclassifiedQuantity: 0 },
};

function RobotDashboardDetails() {

    const { date, machineId } = useParams();

    const [hasDataFinishLoading, setHasDataFinishLoading] = useState(false);
    const [currentProduction, setCurrentProduction] = useState(currentProductionInitialState);
    const [currentMachineData, setCurrentMachineData] = useState({ id: 0, model: "" });

    useEffect(() => {
        axios.get(`${URL}/SensorReadings/${machineId}`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            }
        }).then((res) => {
            setCurrentProduction(res.data);
            setHasDataFinishLoading(true);
            console.log("res", res.data);
        });

        axios.get(`${URL}/Robots/${machineId}`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            }
        }).then((res) => {
            setCurrentMachineData(res.data);
            console.log("res", res.data);
        });

    }, []);

    return hasDataFinishLoading ? (
        <div className="w-full">
            <BackButton className="ml-16" />
            <Typography className="text-center font-bebasNeue pt-16" color="textSecondary" variant="h3">
                {currentMachineData.description}
            </Typography>
            <Typography className="text-center font-bebasNeue pt-16" color="textSecondary" variant="subtitle1">
                {currentProduction.sensorId}
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
                                        {/*{moment(date).format("YYYY-MM-DD")}*/}
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
                                        {/*{currentProduction.leafCountQuantity.totalQuantity}*/}
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
                                        Inicio
                                    </Typography>
                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {/*{moment(firstLogin.loginTime).format("LT")}*/}
                                    </Typography>
                                </div>
                            </div>

                            <div className="flex w-full sm:w-1/2 md:w-1/4 mb-8 md:m-auto">
                                <AlarmOffIcon color="action" fontSize="large" />
                                <div className="mt-4 ml-8">
                                    <Typography style={bandStylesTitle} color="textSecondary">
                                        Fin
                                    </Typography>

                                    <Typography
                                        className="font-bold"
                                        color="textPrimary"
                                        style={bandStylesBody}
                                    >
                                        {/*{lastLogin.logoutTime*/}
                                        {/*    ? moment(lastLogin.logoutTime).format("LT")*/}
                                        {/*    : "N/A"}*/}
                                    </Typography>
                                </div>
                            </div>
                        </div>
                    </div>

                    {/*{stopCauses.length > 0 ? (*/}
                    {/*    <>*/}
                    {/*        <div className="flex flex-col sm:flex-row align-center ml-32 pt-12 mt-32">*/}
                    {/*            <TimerOutlinedIcon color="action" fontSize="large" />*/}
                    {/*            <Typography className="pr-16 text-18 h2 font-bold">Paradas</Typography>*/}
                    {/*        </div>*/}

                    {/*        <GraphicAndTableSelection stopCauses={stopCauses} />*/}
                    {/*    </>*/}
                    {/*) : (*/}
                    {/*    <Typography color="textSecondary" variant="h5" className="mt-128 text-center">*/}
                    {/*        No hay paradas*/}
                    {/*    </Typography>*/}
                    {/*)}*/}
                </div>
            </motion.div>
        </div>
    ) : (
        <ChargingProgressBar />
    );
}

export default withReducer("boothDashboardDetailsApp", reducer)(RobotDashboardDetails);
