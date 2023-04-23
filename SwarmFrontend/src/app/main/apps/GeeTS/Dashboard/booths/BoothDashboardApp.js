import { motion } from "framer-motion";
import { Card, Typography } from "@mui/material";
import { Link } from "react-router-dom";
import BoothWidget from "./BoothWidget";
import useAxiosGetRequest from "../../useAxiosGetRequest";

function BoothDashboardApp() {
    const booths = useAxiosGetRequest("/api/SortingProduction/dashboard/machines", []);

    return (
        <div className="w-full inline-flex m-auto justify-center">
            {booths.length > 0 ? (
                <div className="flex w-full sm:p-8 container inline-flex m-auto justify-center mx-24 ml-96">
                    <motion.div
                        className="widget flex flex-col sm:flex-row flex-wrap w-full p-16 text-center justify-center"
                        enter={{
                            animation: "transition.slideUpBigIn",
                            duration: 300,
                        }}
                    >
                        {booths.map((booth, index) => (
                            <Card key={index} className="h-auto w-512 my-16 sm:m-16 items-center rounded-8">
                                <Link
                                    to={`/apps/GTS/dashboard/booths/${booth.dateCreated}/${booth.id}`}
                                    role="button"
                                >
                                    <BoothWidget booth={booth} />
                                </Link>
                            </Card>
                        ))}
                    </motion.div>
                </div>
            ) : (
                <Typography className="text-center mt-32" variant="h4" color="primary">
                    No hay producción en curso
                </Typography>
            )}
        </div>
    );
}

export default BoothDashboardApp;
