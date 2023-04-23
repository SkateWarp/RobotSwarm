import { styled } from "@mui/material/styles";
import FusePageSimple from "../../../@fuse/core/FusePageSimple/FusePageSimple";

// Este es un hook general de estilo que puede ser utilizado en todos los componentes App
const useGeneralAppStyle = () => {
    return styled(FusePageSimple)(({ theme }) => ({
        "& .FusePageSimple-header": {
            minHeight: 72,
            height: 72,
            [theme.breakpoints.up("lg")]: {
                minHeight: 72,
                height: 72,
            },
        },
        "& .FusePageSimple-wrapper": {
            minHeight: 0,
        },
        "& .FusePageSimple-contentWrapper": {
            padding: 0,
            [theme.breakpoints.up("sm")]: {
                padding: 24,
                height: "100%",
            },
        },
        "& .FusePageSimple-content": {
            display: "flex",
            flexDirection: "column",
            height: "100%",
        },
        "& .FusePageSimple-sidebar": {
            width: 272,
            border: 0,
        },
    }));
};

export default useGeneralAppStyle;
