import withReducer from "app/store/withReducer";
import { useRef } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useParams } from "react-router-dom";
import { useDeepCompareEffect } from "@fuse/hooks";
import useActualProjectName from "app/shared-components/hooks/useActualProjectName";
import AccountDialog from "./AccountDialog";
import AccountsList from "./AccountsList";
import AccountsSidebarContent from "./AccountsSidebarContent";
import reducer from "./store";
import { getAccounts, getAccountsByCompany, setAccountsSearchText } from "./store/accountsSlice";
import GeneralHeader from "../../../shared-components/GeneralHeader";
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

    return (
        <>
            <Root
                header={
                    <GeneralHeader
                        searchText={searchText}
                        pageLayout={pageLayout}
                        headerName="Cuentas"
                        handleSearchTextChange={setAccountsSearchText}
                        iconType="account_box"
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
