/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import SimulationSessionService from "../../../../../services/SimulationSessionService";
import TaskRunHistoryApp from "./TaskRunHistoryApp";

const mockNavigate = jest.fn();

jest.mock("react-router-dom", () => ({
    ...jest.requireActual("react-router-dom"),
    useNavigate: () => mockNavigate,
}));

jest.mock("../../../../../services/SimulationSessionService", () => ({
    __esModule: true,
    default: {
        listTaskHistory: jest.fn(),
    },
}));

describe("TaskRunHistoryApp requests", () => {
    let host;
    let consoleError;

    const flush = async () => {
        await act(async () => {
            await Promise.resolve();
            await Promise.resolve();
            await Promise.resolve();
        });
    };

    const renderApp = () => {
        act(() => {
            ReactDOM.render(<TaskRunHistoryApp />, host);
        });
    };

    const button = (label) =>
        Array.from(document.querySelectorAll("button")).find((candidate) =>
            candidate.textContent.includes(label)
        );

    beforeEach(() => {
        host = document.createElement("div");
        document.body.appendChild(host);
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        mockNavigate.mockReset();
        jest.clearAllMocks();
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        consoleError.mockRestore();
    });

    it("shows a loading state while the owner-scoped history request is pending", () => {
        SimulationSessionService.listTaskHistory.mockReturnValue(new Promise(() => {}));

        renderApp();

        expect(document.body.textContent).toContain("Cargando tareas reales…");
    });

    it("loads the first real TaskRun page and renders an honest empty state", async () => {
        SimulationSessionService.listTaskHistory.mockResolvedValue({ total: 0, items: [] });

        renderApp();
        await flush();

        expect(SimulationSessionService.listTaskHistory).toHaveBeenCalledWith({
            offset: 0,
            limit: 10,
            type: "",
            outcome: "",
        });
        expect(document.body.textContent).toContain("No hay tareas para estos filtros");
        expect(document.querySelector('[aria-label="Historial de tareas ROS"]')).toBeNull();
    });

    it("does not render an empty table on failure and recovers through Reintentar", async () => {
        SimulationSessionService.listTaskHistory
            .mockRejectedValueOnce({ response: { data: { detail: "Historial no disponible." } } })
            .mockResolvedValueOnce({ total: 0, items: [] });

        renderApp();
        await flush();

        expect(document.body.textContent).toContain("Historial no disponible.");
        expect(document.querySelector('[aria-label="Historial de tareas ROS"]')).toBeNull();

        act(() => button("Reintentar").click());
        await flush();

        expect(SimulationSessionService.listTaskHistory).toHaveBeenCalledTimes(2);
        expect(document.body.textContent).toContain("No hay tareas para estos filtros");
        expect(document.body.textContent).not.toContain("Historial no disponible.");
    });
});
