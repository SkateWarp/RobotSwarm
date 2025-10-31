import { useEffect, useState } from "react";
import { motion } from "framer-motion";
import { Card, Typography } from "@mui/material";
import axios from "axios";
import BoothWidget from "./RobotWidget";
import { URL } from "../../../../../constants/constants";
import jwtService from "../../../../../services/jwtService";
import ChargingProgressBar from "app/shared-components/ChargingProgressBar";

function RobotDashboardApp() {
    const [robots, setRobots] = useState([]);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        axios
            .get(`${URL}/Robots`, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${jwtService.getAccessToken()}`,
                },
            })
            .then((response) => {
                setRobots(response.data);
                setLoading(false);
            })
            .catch((error) => {
                console.error("Error fetching robots:", error);
                setLoading(false);
            });
    }, []);

    if (loading) {
        return <ChargingProgressBar />;
    }

    return (
        <div className="w-full inline-flex m-auto justify-center">
            {robots.length > 0 ? (
                <div className="w-full sm:p-8 container m-auto mx-24 ml-96">
                    <motion.div
                        className="grid grid-cols-1 md:grid-cols-3 xl:grid-cols-4 gap-16 p-16"
                        enter={{
                            animation: "transition.slideUpBigIn",
                            duration: 300,
                        }}
                    >
                        {robots.map((robot, index) => (
                            <Card key={index} className="h-auto rounded-8">
                                <BoothWidget robot={robot}/>
                            </Card>
                        ))}
                    </motion.div>
                </div>
            ) : (
                <Typography className="text-center mt-32" variant="h4" color="primary">
                    No hay robots
                </Typography>
            )}
        </div>
    );
}

export default RobotDashboardApp;
