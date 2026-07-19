/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import WhepViewer, { getIceServers } from "./WhepViewer";

jest.mock("./HlsViewer", () => ({
    formatLeaseCountdown: () => "Vence en 2:00",
    isViewerFullscreen: (element) =>
        Boolean(
            element &&
                (global.document.fullscreenElement || global.document.webkitFullscreenElement) === element
        ),
    toggleViewerFullscreen: async (element) => {
        const active =
            (global.document.fullscreenElement || global.document.webkitFullscreenElement) === element;
        if (active) {
            const close = global.document.exitFullscreen || global.document.webkitExitFullscreen;
            await Promise.resolve(close.call(global.document));
            return false;
        }
        const open = element.requestFullscreen || element.webkitRequestFullscreen;
        await Promise.resolve(open.call(element));
        return true;
    },
    useLeaseCountdown: () => 120,
    usePlaybackFps: () => null,
}));

const response = (status, headers = {}, body = "") => ({
    status,
    ok: status >= 200 && status < 300,
    headers: {
        get: (name) => headers[name] || headers[name.toLowerCase()] || null,
    },
    text: jest.fn().mockResolvedValue(body),
});

class PeerConnection {
    constructor() {
        this.iceGatheringState = "complete";
        this.connectionState = "new";
        this.localDescription = { sdp: "local-offer" };
        this.close = jest.fn();
        PeerConnection.instances.push(this);
    }

    addTransceiver() {
        this.transceiverAdded = true;
    }

    createOffer() {
        this.offerCreated = true;
        return Promise.resolve({ type: "offer", sdp: "local-offer" });
    }

    setLocalDescription() {
        this.localDescriptionSet = true;
        return Promise.resolve();
    }

    setRemoteDescription() {
        this.remoteDescriptionSet = true;
        return Promise.resolve();
    }

    addEventListener(name, listener) {
        this.listeners = { ...this.listeners, [name]: listener };
    }

    removeEventListener(name) {
        if (this.listeners) delete this.listeners[name];
    }
}

PeerConnection.instances = [];

const flushPromises = async () => {
    await Promise.resolve();
    await Promise.resolve();
    await Promise.resolve();
};

describe("private WHEP viewer lifecycle", () => {
    let container;
    let consoleError;

    beforeEach(() => {
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        container = document.createElement("div");
        document.body.appendChild(container);
        PeerConnection.instances = [];
        window.RTCPeerConnection = PeerConnection;
        global.fetch = jest.fn((url, options) => {
            if (options.method === "OPTIONS") return Promise.resolve(response(204));
            if (options.method === "POST") {
                return Promise.resolve(
                    response(201, { Location: "/private/whep/session-1" }, "remote-answer")
                );
            }
            return Promise.resolve(response(204));
        });
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(container);
        });
        container.remove();
        delete window.RTCPeerConnection;
        delete global.fetch;
        delete document.webkitExitFullscreen;
        delete document.webkitFullscreenElement;
        jest.useRealTimers();
        const unexpectedErrors = consoleError.mock.calls.filter(
            ([message]) => !String(message).includes("unstable_flushDiscreteUpdates")
        );
        consoleError.mockRestore();
        if (unexpectedErrors.length > 0) {
            throw new Error(`Unexpected React error: ${unexpectedErrors[0][0]}`);
        }
    });

    it("closes the peer and deletes the remote WHEP session on unmount", async () => {
        await act(async () => {
            ReactDOM.render(
                <WhepViewer
                    url="https://robot.example/private/whep"
                    token="lease-token"
                    expiresAt="2099-01-01T00:00:00Z"
                />,
                container
            );
            await flushPromises();
        });

        const peer = PeerConnection.instances[0];
        expect(peer).toBeDefined();

        act(() => {
            ReactDOM.unmountComponentAtNode(container);
        });

        expect(peer.close).toHaveBeenCalledTimes(1);
        expect(global.fetch).toHaveBeenCalledWith(
            "https://robot.example/private/whep/session-1",
            expect.objectContaining({
                method: "DELETE",
                credentials: "omit",
                headers: { Authorization: "Bearer lease-token" },
            })
        );
    });

    it("expires an active viewer, closes it and revokes its WHEP session", async () => {
        jest.useFakeTimers();
        const expiresAt = new Date(Date.now() + 1000).toISOString();

        await act(async () => {
            ReactDOM.render(
                <WhepViewer
                    url="https://robot.example/private/whep"
                    token="lease-token"
                    expiresAt={expiresAt}
                />,
                container
            );
            await flushPromises();
        });

        await act(async () => {
            jest.advanceTimersByTime(1001);
            await flushPromises();
        });

        expect(container.textContent).toContain("El acceso al visor expiró");
        expect(PeerConnection.instances[0].close).toHaveBeenCalledTimes(1);
        expect(global.fetch.mock.calls.some(([, options]) => options.method === "DELETE")).toBe(true);
    });

    it("explains the video-only fallback and toggles WebKit fullscreen", async () => {
        let fullscreenElement = null;
        Object.defineProperty(document, "webkitFullscreenElement", {
            configurable: true,
            get: () => fullscreenElement,
        });

        await act(async () => {
            ReactDOM.render(
                <WhepViewer
                    url="https://robot.example/private/whep"
                    token="lease-token"
                    expiresAt="2099-01-01T00:00:00Z"
                />,
                container
            );
            await flushPromises();
        });

        const viewer = container.querySelector('[data-testid="private-viewer"]');
        viewer.webkitRequestFullscreen = jest.fn(() => {
            fullscreenElement = viewer;
            document.dispatchEvent(new Event("webkitfullscreenchange"));
        });
        document.webkitExitFullscreen = jest.fn(() => {
            fullscreenElement = null;
            document.dispatchEvent(new Event("webkitfullscreenchange"));
        });

        act(() => {
            container.querySelector('[data-testid="viewer-video"]').dispatchEvent(new Event("playing"));
        });
        expect(container.querySelector('[data-testid="whep-fallback-note"]').textContent).toContain(
            "solo video"
        );

        await act(async () => {
            container.querySelector('[aria-label="Abrir visor en pantalla completa"]').click();
            await Promise.resolve();
        });
        expect(container.querySelector('[aria-label="Salir de pantalla completa"]')).not.toBeNull();

        await act(async () => {
            container.querySelector('[aria-label="Salir de pantalla completa"]').click();
            await Promise.resolve();
        });
        expect(container.querySelector('[aria-label="Abrir visor en pantalla completa"]')).not.toBeNull();
    });
});

describe("WHEP ICE links", () => {
    it("parses TURN credentials without placing them in the page", () => {
        expect(getIceServers('<turn:turn.example:3478>; username="robot"; credential="short-lived"')).toEqual([
            {
                urls: ["turn:turn.example:3478"],
                username: "robot",
                credential: "short-lived",
            },
        ]);
    });
});
