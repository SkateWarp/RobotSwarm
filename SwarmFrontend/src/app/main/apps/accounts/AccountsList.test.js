/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import AccountsList from "./AccountsList";
import { reactivateAccount } from "./store/accountsSlice";

const mockDispatch = jest.fn();

const mockState = {
    auth: {
        user: {
            id: 1,
            email: "admin@example.test",
        },
    },
    accountsApp: {
        accounts: {
            error: null,
            loading: false,
            searchText: "",
        },
    },
    testAccounts: [
        {
            id: 2,
            firstName: "Grace",
            lastName: "Hopper",
            email: "grace@example.test",
            role: "User",
            enabled: false,
        },
    ],
};

jest.mock("react-redux", () => ({
    useDispatch: () => mockDispatch,
    useSelector: (selector) => selector(mockState),
}));

jest.mock("./store/accountsSlice", () => ({
    disableAccount: jest.fn((accountId) => ({
        type: "accounts/disable",
        payload: accountId,
    })),
    getAccounts: jest.fn(() => ({ type: "accounts/load" })),
    openEditAccountDialog: jest.fn((account) => ({
        type: "accounts/edit",
        payload: account,
    })),
    reactivateAccount: jest.fn((accountId) => ({
        type: "accounts/reactivate",
        payload: accountId,
    })),
    selectAccounts: (state) => state.testAccounts,
    setAccountsSearchText: jest.fn(() => ({ type: "accounts/search" })),
}));

jest.mock("../../../store/fuse/messageSlice", () => ({
    showMessage: jest.fn((payload) => ({ type: "message/show", payload })),
}));

jest.mock("./AccountsTable", () => {
    const React = jest.requireActual("react");
    return function AccountsTable({ columns, data }) {
        const actions = columns.find((column) => column.id === "actions");
        return (
            <div data-testid="accounts-table">
                {data.map((account) => (
                    <div key={account.id}>{actions.Cell({ row: { original: account } })}</div>
                ))}
            </div>
        );
    };
});

describe("AccountsList lifecycle actions", () => {
    let host;
    let consoleError;

    const flush = async () => {
        await act(async () => {
            await Promise.resolve();
            await Promise.resolve();
            await new Promise((resolve) => setTimeout(resolve, 250));
        });
    };

    const selectOption = (label, optionText) => {
        const field = document.querySelector(`[aria-label="${label}"]`);
        const select = field.querySelector('[role="button"]');
        act(() => {
            select.dispatchEvent(new MouseEvent("mousedown", { bubbles: true }));
        });
        const option = Array.from(document.querySelectorAll('[role="option"]')).find(
            (candidate) => candidate.textContent === optionText
        );
        act(() => {
            option.dispatchEvent(new MouseEvent("click", { bubbles: true }));
        });
    };

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        jest.clearAllMocks();
        reactivateAccount.mockImplementation((accountId) => ({
            type: "accounts/reactivate",
            payload: accountId,
        }));
        mockDispatch.mockImplementation((action) => {
            if (action?.type === "accounts/reactivate") {
                return { unwrap: () => Promise.resolve() };
            }
            return action;
        });
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        consoleError.mockRestore();
    });

    it("reactivates an inactive account after an explicit confirmation", async () => {
        act(() => {
            ReactDOM.render(<AccountsList embedded showSearch />, host);
        });
        selectOption("Filtrar cuentas por estado", "Inactivas");

        const action = document.querySelector('[aria-label="Reactivar la cuenta de Grace Hopper"]');
        expect(action).not.toBeNull();
        act(() => action.click());

        const dialog = document.querySelector('[role="dialog"]');
        expect(dialog.textContent).toContain("Sus sesiones y visores anteriores permanecerán cerrados.");
        const confirm = Array.from(dialog.querySelectorAll("button")).find((button) =>
            button.textContent.includes("Reactivar")
        );
        act(() => confirm.click());
        await flush();

        expect(reactivateAccount).toHaveBeenCalledWith(2);
        expect(mockDispatch).toHaveBeenCalledWith({
            type: "accounts/reactivate",
            payload: 2,
        });
        expect(document.querySelector('[role="dialog"]')).toBeNull();
    });
});
