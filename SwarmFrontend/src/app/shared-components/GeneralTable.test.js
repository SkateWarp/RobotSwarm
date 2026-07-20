/** @jest-environment jsdom */

import { activateTableRowFromKeyboard } from "./GeneralTable";

describe("GeneralTable keyboard rows", () => {
    it.each(["Enter", " "])("opens a focused row with %p", (key) => {
        const row = { id: "row-1" };
        const target = {};
        const event = { key, target, currentTarget: target, preventDefault: jest.fn() };
        const onRowClick = jest.fn();

        activateTableRowFromKeyboard(event, onRowClick, row);

        expect(event.preventDefault).toHaveBeenCalledTimes(1);
        expect(onRowClick).toHaveBeenCalledWith(event, row);
    });

    it("does not hijack keyboard events from controls inside the row", () => {
        const event = { key: "Enter", target: {}, currentTarget: {}, preventDefault: jest.fn() };
        const onRowClick = jest.fn();

        activateTableRowFromKeyboard(event, onRowClick, { id: "row-1" });

        expect(event.preventDefault).not.toHaveBeenCalled();
        expect(onRowClick).not.toHaveBeenCalled();
    });
});
