/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import { Button, Chip } from "@mui/material";
import PageHeading from "./PageHeading";

describe("PageHeading", () => {
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

    it("keeps one semantic page title with status and actions", () => {
        act(() => {
            ReactDOM.render(
                <PageHeading
                    title="Robots"
                    description="Inventario disponible."
                    meta="3 robots"
                    status={<Chip label="Actualizado" />}
                    actions={<Button>Registrar robot</Button>}
                />,
                host
            );
        });

        expect(document.querySelectorAll("h1")).toHaveLength(1);
        expect(document.querySelector("h1").textContent).toBe("Robots");
        expect(document.body.textContent).toContain("Registrar robot");
    });
});
