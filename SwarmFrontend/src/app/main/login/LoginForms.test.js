/** @jest-environment jsdom */

import { combineReducers, configureStore } from "@reduxjs/toolkit";
import { createTheme, ThemeProvider } from "@mui/material/styles";
import { Provider } from "react-redux";
import ReactDOM from "react-dom";
import { act, Simulate } from "react-dom/test-utils";
import axios from "axios";
import history from "@history";
import loginReducer from "../../auth/store/loginSlice";
import JWTLoginTab from "./tabs/JWTLoginTab";
import JWTRegisterTab, { getRegistrationErrorMessage, registrationSchema } from "./tabs/JWTRegisterTab";
import { URL } from "../../constants/constants";

jest.mock("axios", () => ({
    post: jest.fn(),
}));

jest.mock("@history", () => ({
    __esModule: true,
    default: {
        push: jest.fn(),
    },
}));

const theme = createTheme();
const validAccount = {
    firstName: "Ana",
    lastName: "Pérez",
    email: "ana@example.com",
    password: "Clave1!a",
};

describe("Login and public registration forms", () => {
    let host;

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        axios.post.mockReset();
        history.push.mockReset();
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

    const fillValidAccount = async () => {
        await act(async () => {
            Object.entries(validAccount).forEach(([name, value]) => {
                Simulate.change(document.querySelector(`input[name="${name}"]`), {
                    target: { name, value },
                });
            });
            await Promise.resolve();
        });
    };

    it("keeps the password control inside its historical alignment and toggles its type", () => {
        const store = configureStore({
            reducer: {
                auth: combineReducers({
                    login: loginReducer,
                }),
            },
        });
        render(
            <Provider store={store}>
                <JWTLoginTab />
            </Provider>
        );

        const password = document.querySelector('input[name="password"]');
        const toggle = document.querySelector('button[aria-label="Mostrar contraseña"]');

        expect(password.type).toBe("password");
        expect(toggle.type).toBe("button");
        expect(toggle.className).not.toContain("MuiIconButton-edgeEnd");

        act(() => {
            Simulate.click(toggle);
        });

        expect(password.type).toBe("text");
        expect(document.querySelector('button[aria-label="Ocultar contraseña"]')).not.toBeNull();
    });

    it("uses the same account policy as the backend", async () => {
        await expect(registrationSchema.validate(validAccount)).resolves.toEqual(validAccount);
        await expect(
            registrationSchema.validate({ ...validAccount, password: "sin-seguridad" })
        ).rejects.toThrow("Usa entre 8 y 16 caracteres");
        await expect(
            registrationSchema.validate({ ...validAccount, firstName: "A".repeat(101) })
        ).rejects.toThrow("no puede superar 100 caracteres");
    });

    it.each(["user@localhost", "user@-example.com", "usér@example.com"])(
        "rejects an email that the backend would reject: %s",
        async (email) => {
            await expect(registrationSchema.validate({ ...validAccount, email })).rejects.toThrow(
                "El correo electrónico debe ser válido"
            );
        }
    );

    it("posts a valid account once and returns to the login", async () => {
        axios.post.mockResolvedValue({ status: 200 });
        render(<JWTRegisterTab />);

        await fillValidAccount();

        const form = document.querySelector("form");
        const submit = document.querySelector('button[aria-label="Crear cuenta"]');
        expect(submit.disabled).toBe(false);

        await act(async () => {
            Simulate.submit(form);
            await Promise.resolve();
            await Promise.resolve();
        });

        expect(axios.post).toHaveBeenCalledTimes(1);
        expect(axios.post).toHaveBeenCalledWith(`${URL}/Accounts`, validAccount);
        expect(history.push).toHaveBeenCalledWith("/login", { accountCreated: true });
    });

    it("shows a useful message when the registration limit is reached", async () => {
        axios.post.mockRejectedValue({ response: { status: 429 } });
        render(<JWTRegisterTab />);

        await fillValidAccount();

        await act(async () => {
            Simulate.submit(document.querySelector("form"));
            await Promise.resolve();
            await Promise.resolve();
        });

        expect(document.querySelector('[role="alert"]').textContent).toContain(
            "Se alcanzó el límite de registros"
        );
        expect(history.push).not.toHaveBeenCalled();
    });

    it("maps closed registration to a neutral message", () => {
        expect(getRegistrationErrorMessage({ response: { status: 403 } })).toBe(
            "El registro no está disponible en este momento."
        );
    });
});
