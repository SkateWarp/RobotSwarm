/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import AccountsApp from "./AccountsApp";

const mockDispatch = jest.fn();

jest.mock("react-redux", () => ({
    useDispatch: () => mockDispatch,
    useSelector: (selector) =>
        selector({
            accountsApp: { accounts: { searchText: "" } },
        }),
}));
jest.mock("react-router-dom", () => ({ useParams: () => ({ param: "all" }) }));
jest.mock("@fuse/hooks", () => ({ useDeepCompareEffect: () => {} }));
jest.mock("app/store/withReducer", () => () => (Component) => Component);
jest.mock("./store", () => ({}));
jest.mock("./store/accountsSlice", () => ({
    getAccounts: () => ({ type: "accounts/load" }),
    getAccountsByCompany: () => ({ type: "accounts/loadByCompany" }),
    setAccountsSearchText: () => ({ type: "accounts/search" }),
}));
jest.mock("app/shared-components/hooks/useActualProjectName", () => () => "GTS");
jest.mock("../../../shared-components/hooks/useActualCompanyId", () => () => 1);
jest.mock("../../../shared-components/GeneralHeader", () => () => null);
jest.mock("./AccountDialog", () => () => null);
jest.mock("./AccountsList", () => () => null);
jest.mock("./AccountsSidebarContent", () => () => null);
jest.mock("./PanelTempAccountDialog", () => () => null);
jest.mock("./GtsAccountDialog", () => () => null);
jest.mock("../../../shared-components/hooks/useGeneralAppStyle", () => {
    const React = jest.requireActual("react");
    const MockPage = React.forwardRef((props, ref) => <main data-testid={props["data-testid"]} ref={ref} />);
    return () => MockPage;
});

describe("AccountsApp accessibility markers", () => {
    let host;

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        mockDispatch.mockClear();
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
    });

    it("publishes a stable marker for the user-management page", () => {
        act(() => {
            ReactDOM.render(<AccountsApp />, host);
        });

        expect(document.querySelector('[data-testid="accounts-page"]')).not.toBeNull();
    });
});
