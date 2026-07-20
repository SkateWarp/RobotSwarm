import withReducer from "app/store/withReducer";
import { Box, Button } from "@mui/material";
import PersonAddOutlined from "@mui/icons-material/PersonAddOutlined";
import { useRef } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useParams } from "react-router-dom";
import { useDeepCompareEffect } from "@fuse/hooks";
import useActualProjectName from "app/shared-components/hooks/useActualProjectName";
import AccountDialog from "./AccountDialog";
import AccountsList from "./AccountsList";
import AccountsSidebarContent from "./AccountsSidebarContent";
import reducer from "./store";
import {
    getAccounts,
    getAccountsByCompany,
    openNewAccountDialog,
    setAccountsSearchText,
} from "./store/accountsSlice";
import GeneralHeader from "../../../shared-components/GeneralHeader";
import PageHeading from "../../../shared-components/PageHeading";
import useGeneralAppStyle from "../../../shared-components/hooks/useGeneralAppStyle";
import useActualCompanyId from "../../../shared-components/hooks/useActualCompanyId";
import PanelTempAccountDialog from "./PanelTempAccountDialog";
import GtsAccountDialog from "./GtsAccountDialog";

function AccountsApp() {
    const Root = useGeneralAppStyle();

    const dispatch = useDispatch();
    const searchText = useSelector(({ accountsApp }) => accountsApp.accounts.searchText);
    const pageLayout = useRef(null);
    const routeParams = useParams();
    const companyId = useActualCompanyId();
    const actualProjectName = useActualProjectName();
    const isGts = actualProjectName === "GTS" || actualProjectName === "GTS-swedish";

    useDeepCompareEffect(() => {
        if (actualProjectName !== "panelTemp") {
            dispatch(getAccounts(routeParams));
        } else {
            dispatch(getAccountsByCompany({ companyId }));
        }
    }, [routeParams, actualProjectName, companyId]);

    let accountDialog = <AccountDialog />;
    if (actualProjectName === "panelTemp") {
        accountDialog = <PanelTempAccountDialog />;
    } else if (isGts) {
        accountDialog = <GtsAccountDialog />;
    }

    if (isGts) {
        return (
            <>
                <Box
                    data-testid="accounts-page"
                    sx={{ p: { xs: 2, md: 3 }, width: "100%", maxWidth: 1600, mx: "auto" }}
                >
                    <PageHeading
                        title="Usuarios"
                        description="Administre las cuentas autorizadas para operar RobotSwarm."
                        actions={
                            <Button
                                variant="contained"
                                startIcon={<PersonAddOutlined />}
                                onClick={() => dispatch(openNewAccountDialog())}
                            >
                                Crear usuario
                            </Button>
                        }
                    />
                    <AccountsList embedded showSearch />
                </Box>
                {accountDialog}
            </>
        );
    }

    return (
        <>
            <Root
                data-testid="accounts-page"
                header={
                    <GeneralHeader
                        searchText={searchText}
                        pageLayout={pageLayout}
                        headerName="Cuentas"
                        handleSearchTextChange={setAccountsSearchText}
                        iconType="account_box"
                        hasSidebar
                    />
                }
                content={<AccountsList />}
                leftSidebarContent={<AccountsSidebarContent />}
                sidebarInner
                ref={pageLayout}
                innerScroll
            />
            {accountDialog}
        </>
    );
}

export default withReducer("accountsApp", reducer)(AccountsApp);
