/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import LeafTypesConfigApp from "./LeafTypeConfigApp";

const mockDispatch = jest.fn();

jest.mock("react-redux", () => ({
    useDispatch: () => mockDispatch,
}));

jest.mock("app/store/withReducer", () => () => (Component) => Component);
jest.mock("./store", () => ({}));
jest.mock("./store/leafTypeConfigSlice", () => ({
    getTaskTemplates: () => ({ type: "taskTemplates/load" }),
}));
jest.mock("./LeafTypeConfigList", () => () => null);
jest.mock("./LeafTypeConfigDialog", () => () => null);
jest.mock("../../../../shared-components/SimpleGeneralHeader", () => () => null);
jest.mock("../../../../shared-components/hooks/useGeneralAppStyle", () => {
    const React = jest.requireActual("react");
    const MockPage = React.forwardRef((props, ref) => <main data-testid={props["data-testid"]} ref={ref} />);
    return () => MockPage;
});

describe("LeafTypesConfigApp accessibility markers", () => {
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

    it("publishes a stable marker for the task-template page", () => {
        act(() => {
            ReactDOM.render(<LeafTypesConfigApp />, host);
        });

        expect(document.querySelector('[data-testid="task-templates-page"]')).not.toBeNull();
    });
});
