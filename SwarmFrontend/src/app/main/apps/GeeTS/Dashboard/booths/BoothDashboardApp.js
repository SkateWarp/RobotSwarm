import {motion} from "framer-motion";
import {Card, Typography} from "@mui/material";
import {Link} from "react-router-dom";
import BoothWidget from "./BoothWidget";
import useAxiosGetRequest from "../../useAxiosGetRequest";

function BoothDashboardApp() {

    const robots = useAxiosGetRequest("/Robots", []);

    return (
        <div className="w-full inline-flex m-auto justify-center">
            {robots.length > 0 ? (
                <div className="w-full sm:p-8 container inline-flex m-auto justify-center mx-24 ml-96">
                    <motion.div
                        className="widget flex flex-col sm:flex-row flex-wrap w-full p-16 text-center justify-center"
                        enter={{
                            animation: "transition.slideUpBigIn",
                            duration: 300,
                        }}
                    >
                        {robots.map((robot, index) => (
                            <Card key={index} className="h-auto w-512 my-16 sm:m-16 items-center rounded-8">
                                <Link
                                    to={`/apps/GTS/dashboard/booths/`}
                                    role="button"
                                >
                                    <BoothWidget booth={robot}/>
                                </Link>
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

export default BoothDashboardApp;
