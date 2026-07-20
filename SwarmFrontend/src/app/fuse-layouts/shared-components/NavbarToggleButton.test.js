/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import NavbarToggleButton from "./NavbarToggleButton";

const mockDispatch = jest.fn();

jest.mock("@mui/material/useMediaQuery", () => () => true);
jest.mock("react-redux", () => ({
    useDispatch: () => mockDispatch,
    useSelector: (selector) =>
        selector({
            fuse: {
                settings: {
                    current: {
                        layout: { config: { navbar: { style: "style-1" } } },
                    },
                },
            },
        }),
}));

describe("NavbarToggleButton accessibility", () => {
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

    it("announces the navigation toggle without assuming its current state", () => {
        act(() => {
            ReactDOM.render(<NavbarToggleButton />, host);
        });

        expect(document.querySelector('button[aria-label="Alternar navegación principal"]')).not.toBeNull();
    });
});
