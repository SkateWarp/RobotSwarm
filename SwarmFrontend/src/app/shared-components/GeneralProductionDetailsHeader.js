import PropTypes from "prop-types";
import clsx from "clsx";
import { Typography } from "@mui/material";
import moment from "moment";
import { LOGO } from "../constants/constants";

// Header general para los distintos componentes de detalles de las órdenes y algunos reportes. El valor de isTubemill
// es false por default, debido a que no quiero que se muestre el modelo de la máquina en los demás sitios
// que utilizan este componente.
const GeneralProductionDetailsHeader = ({ orderDetail, classes, headerName, isTubeMill = false }) => {
    return (
        <div className="flex flex-col md:flex-row print:flex-row justify-between">
            <div className="flex flex-col justify-center">
                <div className="flex items-center justify-center print:mb-0">
                    <img className="w-180 print:w-160" src={LOGO} alt="logo" />
                </div>
            </div>
            <div
                className={clsx(
                    classes.divider,
                    "mx-32 md:w-px md:h-auto print:w-px print:h-auto print:mx-16"
                )}
            />
            <div className="mt-16 md:mt-0 print:mt-0">
                <Typography color="textSecondary" className="mb-8 font-light text-center" variant="h4">
                    {headerName}
                </Typography>

                {isTubeMill && (
                    <Typography color="textSecondary" className="mb-8 font-light text-center" variant="h4">
                        {orderDetail.machine?.model}
                    </Typography>
                )}
                <table className="w-full">
                    <tbody>
                        <tr>
                            <td className="">
                                <Typography
                                    className="font-light text-center md:text-right print:text-right"
                                    variant="h4"
                                    color="textSecondary"
                                >
                                    ID
                                </Typography>
                            </td>

                            <td className="sm:px-16 print:px-16">
                                <Typography className="font-light" variant="h4">
                                    {orderDetail.id}
                                </Typography>
                            </td>
                        </tr>

                        <tr>
                            <td className="text-center md:text-right print:text-right">
                                <Typography color="textSecondary">FECHA CREACIÓN:</Typography>
                            </td>
                            <td className="sm:px-16 print:px-16">
                                <Typography color="textSecondary">
                                    {orderDetail.createdDateTime &&
                                        moment(orderDetail.createdDateTime).format("DD-MM-YYYY h:mm A")}
                                </Typography>
                            </td>
                        </tr>

                        {orderDetail.startedProductionDateTime &&
                            moment(orderDetail.startedProductionDateTime).format("DD-MM-YYYY") !==
                                "01-01-0001" && (
                                <tr>
                                    <td className="text-center md:text-right print:text-right">
                                        <Typography color="textSecondary">FECHA INICIO PRODUCCIÓN:</Typography>
                                    </td>
                                    <td className="sm:px-16 print:px-16">
                                        <Typography color="textSecondary">
                                            {moment(orderDetail.startedProductionDateTime).format(
                                                "DD-MM-YYYY h:mm A"
                                            )}
                                        </Typography>
                                    </td>
                                </tr>
                            )}
                        {orderDetail.finishProductionDateTime &&
                            moment(orderDetail.finishProductionDateTime).format("DD-MM-YYYY") !==
                                "01-01-0001" && (
                                <tr>
                                    <td className="text-center md:text-right print:text-right">
                                        <Typography color="textSecondary">FECHA FIN PRODUCCION:</Typography>
                                    </td>
                                    <td className="sm:px-16 print:px-16">
                                        <Typography color="textSecondary">
                                            {moment(orderDetail.finishProductionDateTime).format(
                                                "DD-MM-YYYY h:mm A"
                                            )}
                                        </Typography>
                                    </td>
                                </tr>
                            )}
                    </tbody>
                </table>
            </div>
        </div>
    );
};

GeneralProductionDetailsHeader.propTypes = {
    orderDetail: PropTypes.object.isRequired,
    classes: PropTypes.object.isRequired,
    headerName: PropTypes.string.isRequired,
    isTubeMill: PropTypes.bool,
};

export default GeneralProductionDetailsHeader;
