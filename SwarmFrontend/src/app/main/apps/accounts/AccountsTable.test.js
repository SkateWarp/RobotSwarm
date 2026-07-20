/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import AccountsTable from "./AccountsTable";

describe("AccountsTable accessibility", () => {
    let host;

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
    });

    it("gives the user table a stable accessible name", () => {
        const columns = [{ Header: "Nombre", accessor: "name" }];

        act(() => {
            ReactDOM.render(
                <AccountsTable
                    columns={columns}
                    data={[{ id: 1, name: "Cuenta de prueba" }]}
                    onRowClick={() => {}}
                />,
                host
            );
        });

        expect(document.querySelector('table[aria-label="Usuarios"]')).not.toBeNull();
        const row = document.querySelector('tr[aria-label="Editar usuario Cuenta de prueba"]');
        expect(row).not.toBeNull();
        expect(row.getAttribute("aria-haspopup")).toBe("dialog");
    });
});
