import {motion} from "framer-motion";
import {Card, Typography} from "@mui/material";
import BoothWidget from "./RobotWidget";
import useAxiosGetRequest from "../../useAxiosGetRequest";

function RobotDashboardApp() {

    const robots = useAxiosGetRequest("/Robots", []);

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
