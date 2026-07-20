import withReducer from "app/store/withReducer";
import { Box } from "@mui/material";
import { useEffect } from "react";
import { useDispatch } from "react-redux";
import reducer from "./store";
import { getTaskTemplates } from "./store/leafTypeConfigSlice";
import LeafTypesConfigList from "./LeafTypeConfigList";
import LeafTypesConfigDialog from "./LeafTypeConfigDialog";
import PageHeading from "../../../../shared-components/PageHeading";

function LeafTypesConfigApp() {
    const dispatch = useDispatch();

    useEffect(() => {
        dispatch(getTaskTemplates());
    }, [dispatch]);

    return (
        <>
            <Box
                data-testid="task-templates-page"
                sx={{ p: { xs: 2, md: 3 }, width: "100%", maxWidth: 1600, mx: "auto" }}
            >
                <PageHeading
                    title="Plantillas de tareas"
                    description="Catálogo administrativo de algoritmos y parámetros reutilizables."
                />
                <LeafTypesConfigList />
            </Box>
            <LeafTypesConfigDialog />
        </>
    );
}

export default withReducer("leafTypesConfigApp", reducer)(LeafTypesConfigApp);
