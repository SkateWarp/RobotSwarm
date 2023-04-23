/* eslint-disable react-hooks/exhaustive-deps */
import clsx from "clsx";
import { PureComponent, useEffect, useRef, useState } from "react";
import axios from "axios";
import { URL } from "app/constants/constants";
import moment from "moment";
import { useReactToPrint } from "react-to-print";
import { makeStyles } from "@mui/styles";
import { darken } from "@mui/material/styles";
import { useParams } from "react-router-dom";
import {
    Button,
    Card,
    CardContent,
    Table,
    TableCell,
    TableContainer,
    TableHead,
    TableRow,
    Typography,
} from "@mui/material";
import Paper from "@mui/material/Paper/Paper";
import ChargingProgressBar from "../../../../../shared-components/ChargingProgressBar";
import PreparingModal from "../../../../../shared-components/PreparingModal";
import PredictionsDetailsTableBody from "./PredictionsDetailsTableBody";
import PredictionsDetailsHeader from "./PredictionsDetailsHeader";

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

function PredictionsDetailsPage() {
    const classes = useStyles();
    const componentRef = useRef();
    const { date, operatorId, leafName } = useParams();

    const [actualLogin, setActualLogin] = useState({ machine: { id: 0, model: "" } });
    const title = `PREDICCIONES-${leafName}_${moment(date).format("DD-MM-YYYY")}`;
    const [predictions, setPredictions] = useState([]);
    const [leafTypes, setLeafTypes] = useState([]);

    const [status, setStatus] = useState(false);
    const [print, setPrint] = useState(false);

    const handlePrint = useReactToPrint({
        content: () => componentRef.current,
        onAfterPrint: () => {
            setPrint(false);
            document.title = beforeTitle;
        },
    });

    useEffect(() => {
        axios.get(`${URL}/api/LeafCount/predictions/${date}/${operatorId}/${leafName}`).then((res) => {
            setPredictions(res.data);
            setStatus(true);
        });

        axios.get(`${URL}/api/LoginActivityLog/machine/${date}/${operatorId}`).then((res) => {
            setActualLogin(res.data);
        });

        axios.get(`${URL}/api/LeafType`).then((res) => {
            res.data.sort((actualElement, nextElement) => orderByNameAscending(actualElement, nextElement));
            setLeafTypes(res.data);
        });
    }, []);

    const orderByNameAscending = (actualElement, nextElement) => {
        // eslint-disable-next-line no-nested-ternary
        return actualElement.name < nextElement.name ? -1 : actualElement.name === nextElement.name ? 0 : 1;
    };

    class MyReport extends PureComponent {
        render() {
            return predictions.length > 0 ? (
                <Card className="mx-auto w-full print:shadow-none rounded-8 print:px-48 print:pt-32">
                    <CardContent className="py-60 md:p-60 print:p-0">
                        {print && <PreparingModal callBack={handlePrint} />}

                        <PredictionsDetailsHeader
                            actualMachine={actualLogin.machine}
                            productionDate={date}
                            leafName={leafName}
                            predictions={predictions}
                        />

                        <div className="print:mt-28 mt-28">
                            <Typography
                                variant="h6"
                                className="text-32 flex mt-32 mb-16 justify-center"
                                gutterBottom
                                component="div"
                            >
                                CLASIFICACIONES
                            </Typography>

                            <TableContainer component={Paper} className="text-16 mb-36 mt-12">
                                <Table aria-label="simple table">
                                    <TableHead>
                                        <TableRow>
                                            <TableCell className="print:hidden" align="center">
                                                FOTO
                                            </TableCell>
                                            {leafTypes.map((leafType) => (
                                                <TableCell key={leafType.id} align="center">
                                                    {leafType.description}
                                                </TableCell>
                                            ))}
                                        </TableRow>
                                    </TableHead>
                                    {predictions.map((prediction) => (
                                        <PredictionsDetailsTableBody
                                            key={prediction.id}
                                            prediction={prediction}
                                            predictedLeafName={leafName}
                                        />
                                    ))}
                                </Table>
                            </TableContainer>
                        </div>
                    </CardContent>
                </Card>
            ) : (
                <div className="flex flex-1 items-center justify-center h-full">
                    <Typography color="textSecondary" variant="h5">
                        No hay datos!
                    </Typography>
                </div>
            );
        }
    }

    if (!status) {
        return <ChargingProgressBar />;
    }

    return (
        <div className={clsx(classes.root, "flex-grow md:px-64 md:pb-32 flex-shrink-0 p-0 print:p-0")}>
            <Button
                className="my-16 hidden md:inline-flex print:hidden"
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

export default PredictionsDetailsPage;
