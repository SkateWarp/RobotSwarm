/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import {
    createRegistryRobot,
    getRobotRegistryErrorMessage,
    listRegistryRobots,
    reactivateRegistryRobot,
} from "./robotRegistryApi";
import RobotRegistryApp from "./RobotRegistryApp";

const mockNavigate = jest.fn();

jest.mock("react-router-dom", () => ({
    ...jest.requireActual("react-router-dom"),
    useNavigate: () => mockNavigate,
}));

jest.mock("./robotRegistryApi", () => ({
    createRegistryRobot: jest.fn(),
    disableRegistryRobot: jest.fn(),
    getRobotRegistryErrorMessage: jest.fn(),
    listRegistryRobots: jest.fn(),
    reactivateRegistryRobot: jest.fn(),
    updateRegistryRobot: jest.fn(),
}));

describe("RobotRegistryApp", () => {
    let host;
    let consoleError;

    const flush = async () => {
        await act(async () => {
            await Promise.resolve();
            await Promise.resolve();
            await Promise.resolve();
        });
    };

    const finishDialogTransition = async () => {
        await act(async () => {
            await new Promise((resolve) => setTimeout(resolve, 250));
        });
    };

    const renderApp = () => {
        act(() => {
            ReactDOM.render(<RobotRegistryApp />, host);
        });
    };

    const button = (label) =>
        Array.from(document.querySelectorAll("button")).find((candidate) =>
            candidate.textContent.includes(label)
        );

    const setInputValue = (input, value) => {
        const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value").set;
        act(() => {
            setter.call(input, value);
            input.dispatchEvent(new Event("input", { bubbles: true }));
        });
    };

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        mockNavigate.mockReset();
        jest.clearAllMocks();
        getRobotRegistryErrorMessage.mockReturnValue("No fue posible cargar el registro.");
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        consoleError.mockRestore();
    });

    it("keeps a visible loading state while the registry request is pending", () => {
        listRegistryRobots.mockReturnValue(new Promise(() => {}));

        renderApp();

        expect(document.querySelector('[aria-label="Cargando robots"]')).not.toBeNull();
    });

    it("shows an honest empty state after loading an empty registry", async () => {
        listRegistryRobots.mockResolvedValue([]);

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("No hay robots registrados");
        expect(document.body.textContent).not.toContain("No fue posible cargar el registro.");
    });

    it("shows the request error without claiming that the registry is empty and can retry", async () => {
        listRegistryRobots.mockRejectedValueOnce(new Error("offline")).mockResolvedValueOnce([]);

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("No fue posible cargar el registro.");
        expect(document.body.textContent).not.toContain("No hay robots registrados");

        act(() => button("Reintentar").click());
        await flush();

        expect(listRegistryRobots).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("No hay robots registrados");
        expect(document.body.textContent).not.toContain("No fue posible cargar el registro.");
    });

    it("registers a robot through the real component action and refreshes the inventory", async () => {
        listRegistryRobots
            .mockResolvedValueOnce([])
            .mockResolvedValueOnce([{ id: 7, name: "tb3_7", status: 0, isPublic: false }]);
        createRegistryRobot.mockResolvedValue({ id: 7, name: "tb3_7" });

        renderApp();
        await flush();
        act(() => button("Registrar robot").click());

        const nameInput = document.querySelector('[role="dialog"] input');
        expect(nameInput).not.toBeNull();
        setInputValue(nameInput, "  tb3_7  ");
        act(() => button("Guardar").click());
        await flush();
        await finishDialogTransition();

        expect(createRegistryRobot).toHaveBeenCalledWith({
            name: "tb3_7",
            description: null,
            notes: null,
            status: 0,
            isPublic: false,
        });
        expect(listRegistryRobots).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("tb3_7");
        expect(document.querySelector('[role="dialog"]')).toBeNull();
    });

    it("shows disabled inventory and reactivates it through an explicit action", async () => {
        const disabledRobot = {
            id: 9,
            name: "tb3_9",
            status: 2,
            statusDescription: "Disabled",
            isPublic: false,
            isConnected: false,
        };
        listRegistryRobots
            .mockResolvedValueOnce([disabledRobot])
            .mockResolvedValueOnce([{ ...disabledRobot, status: 0, statusDescription: "Idle" }]);
        reactivateRegistryRobot.mockResolvedValue({ ...disabledRobot, status: 0 });

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("1 inactivos");
        const reactivate = document.querySelector('[aria-label="Reactivar tb3_9"]');
        expect(reactivate).not.toBeNull();

        act(() => reactivate.click());
        await flush();

        expect(reactivateRegistryRobot).toHaveBeenCalledWith(disabledRobot);
        expect(listRegistryRobots).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("1 activos");
    });
});
