/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import {
    createRobotGroup,
    getRobotGroupErrorMessage,
    listGroupRobots,
    listRobotGroups,
} from "./robotGroupApi";
import RobotGroupsApp from "./RobotGroupsApp";

const mockNavigate = jest.fn();

jest.mock("react-router-dom", () => ({
    ...jest.requireActual("react-router-dom"),
    useNavigate: () => mockNavigate,
}));

jest.mock("./robotGroupApi", () => ({
    addRobotToGroup: jest.fn(),
    createRobotGroup: jest.fn(),
    deleteRobotGroup: jest.fn(),
    getRobotGroupErrorMessage: jest.fn(),
    listGroupRobots: jest.fn(),
    listRobotGroups: jest.fn(),
    removeRobotFromGroup: jest.fn(),
    updateRobotGroup: jest.fn(),
}));

describe("RobotGroupsApp", () => {
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
            ReactDOM.render(<RobotGroupsApp />, host);
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
        getRobotGroupErrorMessage.mockReturnValue("No fue posible cargar los grupos.");
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        consoleError.mockRestore();
    });

    it("keeps a visible loading state while both inventory requests are pending", () => {
        listRobotGroups.mockReturnValue(new Promise(() => {}));
        listGroupRobots.mockReturnValue(new Promise(() => {}));

        renderApp();

        expect(document.querySelector('[aria-label="Cargando grupos"]')).not.toBeNull();
    });

    it("shows an honest empty state after loading an empty group catalog", async () => {
        listRobotGroups.mockResolvedValue([]);
        listGroupRobots.mockResolvedValue([]);

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("Todavía no hay grupos");
        expect(document.body.textContent).not.toContain("No fue posible cargar los grupos.");
    });

    it("shows the request error without presenting a false empty catalog and can retry", async () => {
        listRobotGroups.mockRejectedValueOnce(new Error("offline")).mockResolvedValueOnce([]);
        listGroupRobots.mockResolvedValue([]);

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("No fue posible cargar los grupos.");
        expect(document.body.textContent).not.toContain("Todavía no hay grupos");

        act(() => button("Reintentar").click());
        await flush();

        expect(listRobotGroups).toHaveBeenCalledTimes(2);
        expect(listGroupRobots).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("Todavía no hay grupos");
    });

    it("creates a group through the real component action and refreshes both catalogs", async () => {
        listRobotGroups
            .mockResolvedValueOnce([])
            .mockResolvedValueOnce([
                { id: 4, name: "Exploradores", description: null, robots: [], dateCreated: null },
            ]);
        listGroupRobots.mockResolvedValue([]);
        createRobotGroup.mockResolvedValue({ id: 4, name: "Exploradores" });

        renderApp();
        await flush();
        act(() => button("Crear grupo").click());

        const nameInput = document.querySelector('[role="dialog"] input');
        expect(nameInput).not.toBeNull();
        setInputValue(nameInput, "  Exploradores  ");
        act(() => button("Guardar").click());
        await flush();
        await finishDialogTransition();

        expect(createRobotGroup).toHaveBeenCalledWith({ name: "Exploradores", description: null });
        expect(listRobotGroups).toHaveBeenCalledTimes(2);
        expect(listGroupRobots).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("Exploradores");
        expect(document.querySelector('[role="dialog"]')).toBeNull();
    });
});
