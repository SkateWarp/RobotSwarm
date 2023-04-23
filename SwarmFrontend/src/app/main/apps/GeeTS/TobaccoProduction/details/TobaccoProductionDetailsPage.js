import clsx from "clsx";
import { PureComponent, useRef, useState } from "react";
import moment from "moment";
import { useReactToPrint } from "react-to-print";
import { makeStyles } from "@mui/styles";
import { darken } from "@mui/material/styles";
import { useParams } from "react-router-dom";
import { Button, Card, CardContent, Typography } from "@mui/material";
import BackButton from "../../../../../fuse-layouts/shared-components/BackButton";
import TobaccoProductionDetailsHeader from "./TobaccoProductionDetailsHeader";
import TobaccoProductionDetailsTable from "./TobaccoProductionDetailsTable";
import ChargingProgressBar from "../../../../../shared-components/ChargingProgressBar";
import PreparingModal from "../../../../../shared-components/PreparingModal";
import useAxiosGetRequest from "../../useAxiosGetRequest";

const useStyles = makeStyles((theme) => ({
    root: {
        background: `radial-gradient(${darken(theme.palette.primary.dark, 0.5)} 0%, ${
            theme.palette.primary.dark
        } 80%)`,
    },
    divider: {
        backgroundColor: theme.palette.divider,
    },
}));

const beforeTitle = document.title;

function TobaccoProductionDetailsPage() {
    const classes = useStyles();
    const componentRef = useRef();
    const { date, shiftId } = useParams();

    const totalProduction = useAxiosGetRequest(
        `/api/SortingProduction/shift/details/${date}/${shiftId}`,
        null
    );
    const title = `REPORTE-_${moment(date).format("DD-MM-YYYY")}`;

    const [print, setPrint] = useState(false);

    const handlePrint = useReactToPrint({
        content: () => componentRef.current,
        onAfterPrint: () => {
            setPrint(false);
            document.title = beforeTitle;
        },
    });

    class MyReport extends PureComponent {
        render() {
            return totalProduction ? (
                <Card className="mx-auto w-full print:shadow-none rounded-8 print:px-48 print:pt-32">
                    <CardContent className="py-60 md:p-60 print:p-0">
                        {print && <PreparingModal callBack={handlePrint} />}

                        <TobaccoProductionDetailsHeader
                            productionDate={date}
                            headerName="Producción de Hojas"
                        />

                        <div className="mt-24">
                            <Typography color="textSecondary" className="text-16">
                                <b>Cantidad Total de hojas:</b>{" "}
                                {totalProduction.leafCountQuantity?.totalQuantity}
                            </Typography>
                        </div>

                        <div>
                            <Typography color="textSecondary" className="text-16">
                                <b>Cantidad Total de hojas clasificadas:</b>{" "}
                                {totalProduction.leafCountQuantity?.classifiedQuantity}
                            </Typography>
                        </div>

                        <div className="mb-24">
                            <Typography color="textSecondary" className="text-16">
                                <b>Cantidad Total de hojas no clasificadas:</b>{" "}
                                {totalProduction.leafCountQuantity?.unclassifiedQuantity}
                            </Typography>
                        </div>

                        {totalProduction.totalSortingProductions?.map((totalSortingProductions, index) => (
                            <TobaccoProductionDetailsTable
                                key={index}
                                totalTobaccoProduction={totalSortingProductions}
                            />
                        ))}
                    </CardContent>
                </Card>
            ) : null;
        }
    }

    if (!totalProduction) {
        return <ChargingProgressBar />;
    }

    return (
        <div className={clsx(classes.root, "flex-grow md:px-64 md:pb-32 flex-shrink-0 p-0 print:p-0")}>
            <BackButton className="my-32" />
            <Button
                className="mt-48 hidden md:inline-flex print:hidden"
                style={{ marginLeft: "3.3%" }}
                variant="contained"
                size="medium"
                color="primary"
                onClick={() => {
                    setPrint(true);
                    document.title = title;
                }}
            >
                Imprimir
            </Button>
            <MyReport ref={componentRef} />
        </div>
    );
}

export default TobaccoProductionDetailsPage;
