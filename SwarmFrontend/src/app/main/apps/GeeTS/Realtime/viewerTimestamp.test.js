import { normalizeViewerTimestamp, parseViewerTimestamp } from "./viewerTimestamp";

describe("viewer API timestamps", () => {
    it("parses a timezone-less .NET value as the same instant as its UTC form", () => {
        const timestamp = "2026-07-21T21:00:00.1234567";
        const parsed = parseViewerTimestamp(timestamp);

        expect(Number.isFinite(parsed)).toBe(true);
        expect(parsed).toBe(Date.UTC(2026, 6, 21, 21, 0, 0, 123));
    });

    it("treats a .NET timestamp without a zone as UTC", () => {
        const parse = jest.spyOn(Date, "parse").mockReturnValue(1784667600000);

        expect(parseViewerTimestamp("2026-07-21T21:00:00.1234567")).toBe(1784667600000);
        expect(parse).toHaveBeenCalledWith("2026-07-21T21:00:00.1234567Z");

        parse.mockRestore();
    });

    it.each(["2026-07-21T21:00:00Z", "2026-07-21T21:00:00+00:00", "2026-07-21T17:00:00-04:00"])(
        "keeps an explicit timezone unchanged: %s",
        (timestamp) => {
            expect(normalizeViewerTimestamp(timestamp)).toBe(timestamp);
        }
    );

    it("keeps invalid and absent values invalid", () => {
        expect(Number.isNaN(parseViewerTimestamp(undefined))).toBe(true);
        expect(Number.isNaN(parseViewerTimestamp("not-a-date"))).toBe(true);
    });
});
