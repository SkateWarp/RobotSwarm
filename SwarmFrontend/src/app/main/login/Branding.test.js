/** @jest-environment jsdom */

import fs from "fs";
import path from "path";
import { createTheme, ThemeProvider } from "@mui/material/styles";
import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import { MemoryRouter } from "react-router-dom";
import Logo from "../../fuse-layouts/shared-components/Logo";
import Login from "./Login";
import LoginConfig from "./LoginConfig";
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
            ReactDOM.render(
                <MemoryRouter>
                    <ThemeProvider theme={theme}>{component}</ThemeProvider>
                </MemoryRouter>,
                host
            );
        });
    };

    it("uses the previous PNG in the shared application logo", () => {
        render(<Logo />);

        const image = document.querySelector('img[alt="RobotSwarm"]');
        expect(image).not.toBeNull();
        expect(image.getAttribute("src")).toBe(LOGO_PATH);
    });

    it.each([
        ["login", Login, "BIENVENIDO"],
        ["register", Register, "CREAR CUENTA"],
    ])("keeps one page heading and the PNG on %s", (_name, Component, title) => {
        render(<Component />);

        expect(document.querySelectorAll("h1")).toHaveLength(1);
        expect(document.querySelector("h1").textContent).toBe(title);
        expect(document.querySelector('img[alt="RobotSwarm"]').getAttribute("src")).toBe(LOGO_PATH);
    });

    it("restores the compact original login card and password recovery link", () => {
        render(<Login />);

        expect(document.querySelector(".max-w-384")).not.toBeNull();
        expect(document.querySelectorAll('img[alt="RobotSwarm"]')).toHaveLength(2);
        expect(document.querySelector('a[href="/forgot-password"]').textContent).toContain(
            "Restablecer contraseña"
        );
        expect(document.querySelector('a[href="/register"]').textContent).toContain("Crear cuenta");
    });

    it("serves the registration page instead of redirecting it", () => {
        const registerRoute = LoginConfig.routes.find((route) => route.path === "/register");

        expect(registerRoute).toBeDefined();
        expect(registerRoute.element.type).toBe(Register);
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
