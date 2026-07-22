/** @jest-environment jsdom */

import fs from "fs";
import path from "path";
import { createTheme, ThemeProvider } from "@mui/material/styles";
import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import Logo from "../../fuse-layouts/shared-components/Logo";
import Login from "./Login";
import Register from "./Register";

jest.mock("./tabs/JWTLoginTab", () => () => <div />);
jest.mock("./tabs/JWTRegisterTab", () => () => <div />);

const LOGO_PATH = "assets/images/logos/logo.png";
const theme = createTheme();

describe("RobotSwarm branding", () => {
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

    const render = (component) => {
        act(() => {
            ReactDOM.render(<ThemeProvider theme={theme}>{component}</ThemeProvider>, host);
        });
    };

    it("uses the previous PNG in the shared application logo", () => {
        render(<Logo />);

        const image = document.querySelector('img[alt="RobotSwarm"]');
        expect(image).not.toBeNull();
        expect(image.getAttribute("src")).toBe(LOGO_PATH);
    });

    it.each([
        ["login", Login, "Iniciar sesión"],
        ["register", Register, "Crear cuenta"],
    ])("keeps one page heading and the PNG on %s", (_name, Component, title) => {
        render(<Component />);

        expect(document.querySelectorAll("h1")).toHaveLength(1);
        expect(document.querySelector("h1").textContent).toBe(title);
        expect(document.querySelector('img[alt="RobotSwarm"]').getAttribute("src")).toBe(LOGO_PATH);
    });

    it("keeps the splash logo legible and exposes the same asset through the manifest", () => {
        const publicRoot = path.join(process.cwd(), "public");
        const index = fs.readFileSync(path.join(publicRoot, "index.html"), "utf8");
        const manifest = JSON.parse(fs.readFileSync(path.join(publicRoot, "manifest.json"), "utf8"));
        const image = fs.readFileSync(path.join(publicRoot, LOGO_PATH));

        expect(index).toContain(`src="${LOGO_PATH}"`);
        expect(index).toContain("filter: invert(1)");
        expect(manifest.icons).toEqual([
            expect.objectContaining({ src: LOGO_PATH, sizes: "2360x1640", type: "image/png" }),
        ]);
        expect(image.readUInt32BE(16)).toBe(2360);
        expect(image.readUInt32BE(20)).toBe(1640);
    });
});
