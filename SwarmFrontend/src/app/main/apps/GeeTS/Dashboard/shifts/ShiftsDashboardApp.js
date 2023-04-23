import { motion } from "framer-motion";
import { Card, Typography } from "@mui/material";
import withReducer from "../../../../../store/withReducer";
import reducer from "../store/boothDashboardSlice";
import ShiftsDashboardData from "./ShiftsDashboardData";
import history from "../../../../../../@history/@history";
import useAxiosGetRequest from "../../useAxiosGetRequest";

function ShiftsDashboardApp() {
    const productions = useAxiosGetRequest("/api/SortingProduction/dashboard/shifts", []);

    const goToDetails = (date, shiftId) => {
        history.push({
            pathname: `/apps/GTS/dashboard/shifts/${date}/${shiftId}`,
        });
    };

    return (
        <motion.div initial={{ y: 10 }} animate={{ y: 0, transition: { delay: 2 } }}>
            {productions.length > 0 ? (
                <div className="flex flex-col w-full inline-flex m-auto justify-center">
                    <Typography className="font-oswald text-center my-16 pb-10" color="primary" variant="h4">
                        Información de Turnos
                    </Typography>

                    {productions.map((production) => (
                        <div
                            key={production.shift?.id}
                            className="flex m-auto justify-center w-full sm:w-7/12"
                        >
                            <Card
                                role="button"
                                onClick={() => {
                                    goToDetails(production.dateCreated, production.shift?.id);
                                }}
                                className="w-full items-center rounded-20 shadow mb-8 sm:mb-0"
                            >
                                <div className="p-4 pt-16 pb-16">
                                    <ShiftsDashboardData production={production} />
                                </div>
                            </Card>
                        </div>
                    ))}
                </div>
            ) : (
                <Typography className="text-center mt-32" variant="h4" color="primary">
                    No hay producción en curso
                </Typography>
            )}
        </motion.div>
    );
}

export default withReducer("shiftsDashboardApp", reducer)(ShiftsDashboardApp);
