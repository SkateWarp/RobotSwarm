import { configureHlsRequest } from "./HlsViewer";

describe("private HLS requests", () => {
    it("adds the lease bearer token without browser credentials", () => {
        const headers = {};
        const request = {
            withCredentials: true,
            setRequestHeader: (name, value) => {
                headers[name] = value;
            },
        };

        configureHlsRequest("short-lived-lease-token")(request);

        expect(request.withCredentials).toBe(false);
        expect(headers).toEqual({
            Authorization: "Bearer short-lived-lease-token",
            "Cache-Control": "no-cache",
        });
    });

    it("uses the same authorization setup for every playlist and part request", () => {
        const setup = configureHlsRequest("one-session-token");
        const requests = [
            { setRequestHeader: jest.fn() },
            { setRequestHeader: jest.fn() },
            { setRequestHeader: jest.fn() },
        ];

        requests.forEach(setup);

        requests.forEach((request) => {
            expect(request.setRequestHeader).toHaveBeenCalledWith("Authorization", "Bearer one-session-token");
        });
    });
});
