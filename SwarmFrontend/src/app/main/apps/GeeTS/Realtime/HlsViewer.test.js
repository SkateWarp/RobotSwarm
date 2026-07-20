/** @jest-environment jsdom */

import ReactDOM from "react-dom";
import { act } from "react-dom/test-utils";
import HlsViewer, {
    configureHlsRequest,
    createHlsRetryWindow,
    createViewerInputQueue,
    formatLeaseCountdown,
    hlsFailureMessage,
    hlsNetworkFailure,
    isViewerKeyCode,
    isViewerFullscreen,
    leaseSecondsRemaining,
    playbackFps,
    toggleViewerFullscreen,
    viewerCoordinates,
    VIEWER_MAX_PENDING_TRANSITIONS,
    VIEWER_INPUT_INTERVAL_MS,
    VIEWER_STARTUP_BUDGET_MS,
} from "./HlsViewer";

const mockHlsInstances = [];

jest.mock("hls.js", () => {
    class MockHls {
        constructor() {
            this.listeners = {};
            mockHlsInstances.push(this);
        }

        on(event, listener) {
            this.listeners[event] = listener;
        }

        attachMedia(media) {
            this.media = media;
            this.listeners[MockHls.Events.MEDIA_ATTACHED]?.();
        }

        loadSource(source) {
            this.source = source;
        }

        recoverMediaError() {
            this.recovered = true;
        }

        destroy() {
            this.destroyed = true;
        }
    }

    MockHls.isSupported = () => true;
    MockHls.Events = {
        MEDIA_ATTACHED: "mediaAttached",
        MANIFEST_PARSED: "manifestParsed",
        ERROR: "error",
    };
    MockHls.ErrorTypes = {
        NETWORK_ERROR: "networkError",
        MEDIA_ERROR: "mediaError",
    };

    return { __esModule: true, default: MockHls };
});

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

describe("viewer measurements", () => {
    it("formats a short-lived lease without exposing its token", () => {
        const now = Date.parse("2026-07-19T12:00:00.000Z");
        const remaining = leaseSecondsRemaining("2026-07-19T12:02:05.000Z", now);

        expect(remaining).toBe(125);
        expect(formatLeaseCountdown(remaining)).toBe("Vence en 2:05");
        expect(formatLeaseCountdown(0)).toBe("Acceso expirado");
    });

    it("calculates decoded playback frames per second", () => {
        expect(playbackFps(30, 1000)).toBe(30);
        expect(playbackFps(75, 2500)).toBe(30);
        expect(playbackFps(0, 1000)).toBeNull();
    });
});

describe("interactive viewer input", () => {
    it("maps pointer coordinates only inside the contained video image", () => {
        const video = {
            videoWidth: 1280,
            videoHeight: 720,
            getBoundingClientRect: () => ({ left: 0, top: 0, width: 1000, height: 1000 }),
        };

        expect(viewerCoordinates(video, 500, 500)).toEqual({ x: 0.5, y: 0.5 });
        expect(viewerCoordinates(video, 500, 218.75)).toEqual({ x: 0.5, y: 0 });
        expect(viewerCoordinates(video, 500, 781.25)).toEqual({ x: 0.5, y: 1 });
        expect(viewerCoordinates(video, 500, 200)).toBeNull();
    });

    it("allows navigation controls but rejects arbitrary browser shortcut codes", () => {
        expect(isViewerKeyCode("KeyW")).toBe(true);
        expect(isViewerKeyCode("ArrowLeft")).toBe(true);
        expect(isViewerKeyCode("F12")).toBe(true);
        expect(isViewerKeyCode("LaunchCalculator")).toBe(false);
        expect(isViewerKeyCode("BrowserBack")).toBe(false);
    });

    it("keeps transitions ahead of coalesced motion and stays below the backend rate limit", () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                jobs.push({ callback, at: time + delay });
                jobs.sort((left, right) => left.at - right.at);
                return jobs.length;
            },
        };
        const runUntil = (deadline) => {
            while (jobs.length > 0 && jobs[0].at < deadline) {
                const job = jobs.shift();
                time = job.at;
                job.callback();
            }
        };
        const deliver = jest.fn();
        const queue = createViewerInputQueue(deliver, jest.fn(), timerObject, () => time);

        for (let index = 0; index < 200; index += 1) {
            queue.enqueue({ type: "pointerMove", x: index / 200, y: 0.5 });
            queue.enqueue({ type: "wheel", x: 0.5, y: 0.5, deltaX: 0, deltaY: 1 });
        }
        queue.enqueue({ type: "keyDown", code: "KeyW" });
        queue.enqueue({ type: "keyUp", code: "KeyW" });
        for (let index = 0; index < 60; index += 1) {
            queue.enqueue({ type: index % 2 === 0 ? "keyDown" : "keyUp", code: "KeyA" });
        }

        runUntil(1000);

        expect(deliver).toHaveBeenCalledTimes(64);
        expect(deliver.mock.calls[0][0]).toEqual({ type: "keyDown", code: "KeyW" });
        expect(deliver.mock.calls[1][0]).toEqual({ type: "keyUp", code: "KeyW" });
        expect(time).toBeGreaterThanOrEqual((deliver.mock.calls.length - 1) * VIEWER_INPUT_INTERVAL_MS);
    });

    it("drops queued motion after a rejection but still sends a requested release", async () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                jobs.push({ callback, at: time + delay });
                jobs.sort((left, right) => left.at - right.at);
                return jobs.length;
            },
        };
        const runNext = () => {
            const job = jobs.shift();
            time = job.at;
            job.callback();
        };
        const deliver = jest
            .fn()
            .mockRejectedValueOnce(new Error("authorization expired"))
            .mockResolvedValue(undefined);
        const failed = jest.fn();
        const queue = createViewerInputQueue(deliver, failed, timerObject, () => time);

        queue.enqueue({ type: "keyDown", code: "KeyW" });
        runNext();
        await expect(deliver.mock.results[0].value).rejects.toThrow("authorization expired");
        queue.enqueue({ type: "pointerMove", x: 0.9, y: 0.9 });
        queue.release([{ type: "releaseAll" }]);
        runNext();

        expect(failed).toHaveBeenCalledTimes(1);
        expect(deliver.mock.calls.map(([input]) => input.type)).toEqual(["keyDown", "releaseAll"]);
    });

    it("keeps only one acknowledged invocation in flight", async () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                const job = { callback, at: time + delay };
                jobs.push(job);
                jobs.sort((left, right) => left.at - right.at);
                return job;
            },
            clearTimeout: (job) => {
                const index = jobs.indexOf(job);
                if (index >= 0) jobs.splice(index, 1);
            },
        };
        const runNext = () => {
            const job = jobs.shift();
            time = job.at;
            job.callback();
        };
        let finishFirst;
        const deliver = jest
            .fn()
            .mockImplementationOnce(
                () =>
                    new Promise((resolve) => {
                        finishFirst = resolve;
                    })
            )
            .mockResolvedValue(undefined);
        const queue = createViewerInputQueue(deliver, jest.fn(), timerObject, () => time);

        queue.enqueue({ type: "keyDown", code: "KeyW" });
        queue.enqueue({ type: "keyUp", code: "KeyW" });
        runNext();
        expect(deliver).toHaveBeenCalledTimes(1);
        expect(jobs.some((job) => job.at < 1000)).toBe(false);

        finishFirst();
        await Promise.resolve();
        runNext();
        expect(deliver).toHaveBeenCalledTimes(2);
    });

    it("times out an unacknowledged invocation and follows it with one releaseAll", () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                const job = { callback, at: time + delay };
                jobs.push(job);
                jobs.sort((left, right) => left.at - right.at);
                return job;
            },
            clearTimeout: (job) => {
                const index = jobs.indexOf(job);
                if (index >= 0) jobs.splice(index, 1);
            },
        };
        const runNext = () => {
            const job = jobs.shift();
            time = job.at;
            job.callback();
        };
        const deliver = jest
            .fn()
            .mockImplementationOnce(() => new Promise(() => {}))
            .mockResolvedValue(undefined);
        const failed = jest.fn();
        const queue = createViewerInputQueue(deliver, failed, timerObject, () => time);

        queue.enqueue({ type: "keyDown", code: "KeyW" });
        runNext();
        runNext();
        runNext();

        expect(failed).toHaveBeenCalledTimes(1);
        expect(deliver.mock.calls.map(([input]) => input.type)).toEqual(["keyDown", "releaseAll"]);
    });

    it("fails closed with one priority release instead of growing transitions", () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                jobs.push({ callback, at: time + delay });
                jobs.sort((left, right) => left.at - right.at);
                return jobs.length;
            },
        };
        const failed = jest.fn();
        const deliver = jest.fn();
        const queue = createViewerInputQueue(deliver, failed, timerObject, () => time);

        for (let index = 0; index <= VIEWER_MAX_PENDING_TRANSITIONS; index += 1) {
            queue.enqueue({ type: index % 2 === 0 ? "keyDown" : "keyUp", code: "KeyA" });
        }
        const job = jobs.shift();
        time = job.at;
        job.callback();

        expect(failed).toHaveBeenCalledTimes(1);
        expect(deliver).toHaveBeenCalledTimes(1);
        expect(deliver).toHaveBeenCalledWith({ type: "releaseAll" });
    });

    it("lets one releaseAll supersede every queued transition", () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                jobs.push({ callback, at: time + delay });
                return jobs.length;
            },
        };
        const deliver = jest.fn();
        const queue = createViewerInputQueue(deliver, jest.fn(), timerObject, () => time);

        queue.enqueue({ type: "keyDown", code: "KeyW" });
        queue.enqueue({ type: "pointerDown", x: 0.5, y: 0.5, button: 0 });
        queue.release([{ type: "releaseAll" }, { type: "releaseAll" }]);
        const job = jobs.shift();
        time = job.at;
        job.callback();

        expect(deliver.mock.calls).toEqual([[{ type: "releaseAll" }]]);
    });

    it("does not enqueue a second releaseAll while the first is in flight", async () => {
        let time = 0;
        const jobs = [];
        const timerObject = {
            setTimeout: (callback, delay) => {
                const job = { callback, at: time + delay };
                jobs.push(job);
                jobs.sort((left, right) => left.at - right.at);
                return job;
            },
            clearTimeout: (job) => {
                const index = jobs.indexOf(job);
                if (index >= 0) jobs.splice(index, 1);
            },
        };
        let finishRelease;
        const deliver = jest.fn(
            () =>
                new Promise((resolve) => {
                    finishRelease = resolve;
                })
        );
        const queue = createViewerInputQueue(deliver, jest.fn(), timerObject, () => time);

        queue.release([{ type: "releaseAll" }]);
        const firstJob = jobs.shift();
        time = firstJob.at;
        firstJob.callback();
        queue.release([{ type: "releaseAll" }, { type: "releaseAll" }]);
        expect(deliver).toHaveBeenCalledTimes(1);

        finishRelease();
        await Promise.resolve();

        expect(deliver).toHaveBeenCalledTimes(1);
        expect(jobs).toEqual([]);
    });
});

describe("HLS startup failures", () => {
    it("keeps a startup budget longer than the worker command timeout", () => {
        expect(VIEWER_STARTUP_BUDGET_MS).toBeGreaterThanOrEqual(30000);
        expect(VIEWER_STARTUP_BUDGET_MS).toBeGreaterThan(15000);
    });

    it.each([
        [404, "pending"],
        [429, "saturated"],
        [502, "origin"],
        [504, "origin"],
        [503, "network"],
    ])("classifies HTTP %i as %s", (statusCode, failure) => {
        expect(hlsNetworkFailure(statusCode)).toBe(failure);
    });

    it("gives a distinct action for a missing playlist, saturation, and an unavailable origin", () => {
        expect(hlsFailureMessage("pending", true)).toContain("lista HLS");
        expect(hlsFailureMessage("saturated", true)).toContain("HTTP 429");
        expect(hlsFailureMessage("origin", true)).toContain("HTTP 502/504");
    });

    it("does not renew the initial budget while startup keeps failing", () => {
        let now = 1000;
        const retryWindow = createHlsRetryWindow(() => now);

        now += 29000;
        expect(retryWindow.beginOrContinueFailure()).toBe(1000);
        now += 1001;
        expect(retryWindow.beginOrContinueFailure()).toBe(0);
        now += 30000;
        expect(retryWindow.beginOrContinueFailure()).toBe(0);
    });

    it("opens one fresh recovery window after successful playback", () => {
        let now = 1000;
        const retryWindow = createHlsRetryWindow(() => now);
        retryWindow.markPlaybackStarted();

        now += 120000;
        expect(retryWindow.beginOrContinueFailure()).toBe(VIEWER_STARTUP_BUDGET_MS);
        now += VIEWER_STARTUP_BUDGET_MS - 1;
        expect(retryWindow.beginOrContinueFailure()).toBe(1);
        now += 2;
        expect(retryWindow.beginOrContinueFailure()).toBe(0);
    });

    it("renews recovery again only after playback actually resumes", () => {
        let now = 1000;
        const retryWindow = createHlsRetryWindow(() => now);
        retryWindow.markPlaybackStarted();
        now += 60000;
        retryWindow.beginOrContinueFailure();
        now += VIEWER_STARTUP_BUDGET_MS + 1;
        expect(retryWindow.beginOrContinueFailure()).toBe(0);

        retryWindow.markPlaybackStarted();
        now += 5000;
        expect(retryWindow.beginOrContinueFailure()).toBe(VIEWER_STARTUP_BUDGET_MS);
    });
});

describe("mounted interactive HLS viewer", () => {
    let host;
    let consoleError;
    let loadMock;
    let playMock;

    const advance = async (milliseconds) => {
        await act(async () => {
            jest.advanceTimersByTime(milliseconds);
            await Promise.resolve();
            await Promise.resolve();
        });
    };

    const renderLiveViewer = async (onInput, expiresAt = "2099-01-01T00:00:00Z") => {
        await act(async () => {
            ReactDOM.render(
                <HlsViewer
                    url="https://robot.example/private/view/index.m3u8"
                    token="short-lived-token"
                    expiresAt={expiresAt}
                    interactiveAvailable
                    onInput={onInput}
                />,
                host
            );
            await Promise.resolve();
        });

        const video = host.querySelector('[data-testid="viewer-video"]');
        Object.defineProperty(video, "videoWidth", { configurable: true, value: 1280 });
        Object.defineProperty(video, "videoHeight", { configurable: true, value: 720 });
        video.getBoundingClientRect = () => ({ left: 0, top: 0, width: 1280, height: 720 });

        act(() => {
            video.dispatchEvent(new Event("playing"));
        });
        act(() => {
            host.querySelector('[aria-label="Activar control interactivo"]').click();
        });
        await advance(1);
        return video;
    };

    const pressKey = (video, type, code) => {
        video.dispatchEvent(
            new KeyboardEvent(type, {
                bubbles: true,
                cancelable: true,
                code,
                key: code === "KeyW" ? "w" : code,
            })
        );
    };

    beforeEach(() => {
        jest.useFakeTimers();
        mockHlsInstances.length = 0;
        host = document.createElement("div");
        document.body.appendChild(host);
        consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        loadMock = jest.spyOn(HTMLMediaElement.prototype, "load").mockImplementation(() => {});
        playMock = jest.spyOn(HTMLMediaElement.prototype, "play").mockImplementation(() => Promise.resolve());
    });

    afterEach(() => {
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        delete document.visibilityState;
        jest.clearAllTimers();
        jest.useRealTimers();
        loadMock.mockRestore();
        playMock.mockRestore();
        const unexpectedErrors = consoleError.mock.calls.filter(
            ([message]) => !String(message).includes("unstable_flushDiscreteUpdates")
        );
        consoleError.mockRestore();
        expect(unexpectedErrors).toEqual([]);
    });

    it("releases a held key when focus leaves the video", async () => {
        const onInput = jest.fn().mockResolvedValue(undefined);
        const video = await renderLiveViewer(onInput);

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(15);
        expect(onInput).toHaveBeenLastCalledWith({ type: "keyDown", code: "KeyW" });

        act(() => {
            host.querySelector('[aria-label="Abrir visor en pantalla completa"]').focus();
        });
        await advance(15);

        expect(onInput).toHaveBeenLastCalledWith({ type: "releaseAll" });
    });

    it("fails closed when the browser hides the viewer tab", async () => {
        const onInput = jest.fn().mockResolvedValue(undefined);
        const video = await renderLiveViewer(onInput);
        Object.defineProperty(document, "visibilityState", {
            configurable: true,
            value: "hidden",
        });

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(15);
        act(() => {
            document.dispatchEvent(new Event("visibilitychange"));
        });
        await advance(15);

        expect(onInput).toHaveBeenLastCalledWith({ type: "releaseAll" });
        expect(host.querySelector('[aria-label="Activar control interactivo"]')).not.toBeNull();
    });

    it("fails closed locally and still attempts the release after an acknowledged rejection", async () => {
        const onInput = jest
            .fn()
            .mockRejectedValueOnce(new Error("control authorization expired"))
            .mockResolvedValue(undefined);
        const video = await renderLiveViewer(onInput);

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(2);
        await advance(15);

        expect(onInput.mock.calls.map(([input]) => input.type)).toEqual(["keyDown", "releaseAll"]);
        expect(host.textContent).toContain("Se perdió el canal de control");
        const reactivate = host.querySelector('[aria-label="Activar control interactivo"]');
        expect(reactivate).not.toBeNull();

        act(() => {
            reactivate.click();
        });
        await advance(1);
        act(() => {
            pressKey(video, "keydown", "KeyA");
        });
        await advance(15);

        expect(onInput).toHaveBeenLastCalledWith({ type: "keyDown", code: "KeyA" });
    });

    it("keeps one input queue when the delivery callback changes", async () => {
        let finishFirstInput;
        const firstInput = jest.fn(
            () =>
                new Promise((resolve) => {
                    finishFirstInput = resolve;
                })
        );
        const secondInput = jest.fn().mockResolvedValue(undefined);
        const video = await renderLiveViewer(firstInput);

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(15);
        expect(firstInput).toHaveBeenCalledTimes(1);

        await act(async () => {
            ReactDOM.render(
                <HlsViewer
                    url="https://robot.example/private/view/index.m3u8"
                    token="short-lived-token"
                    expiresAt="2099-01-01T00:00:00Z"
                    interactiveAvailable
                    onInput={secondInput}
                />,
                host
            );
            await Promise.resolve();
        });
        act(() => {
            pressKey(video, "keyup", "KeyW");
        });
        await advance(15);

        expect(secondInput).not.toHaveBeenCalled();
        finishFirstInput();
        await advance(1);
        await advance(15);

        expect(secondInput).toHaveBeenCalledWith({ type: "keyUp", code: "KeyW" });
    });

    it("sends one global release when the active viewer unmounts", async () => {
        const onInput = jest.fn().mockResolvedValue(undefined);
        const video = await renderLiveViewer(onInput);

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(15);
        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        await advance(15);

        expect(onInput).toHaveBeenLastCalledWith({ type: "releaseAll" });
    });

    it("releases and disables input five seconds before the viewer lease expires", async () => {
        const start = new Date("2026-07-19T12:00:00.000Z");
        jest.setSystemTime(start);
        const expiresAt = new Date(start.getTime() + 7000).toISOString();
        const onInput = jest.fn().mockResolvedValue(undefined);
        const video = await renderLiveViewer(onInput, expiresAt);

        act(() => {
            pressKey(video, "keydown", "KeyW");
        });
        await advance(15);
        await advance(2000);
        await advance(15);

        expect(onInput.mock.calls.map(([input]) => input.type)).toEqual(["keyDown", "releaseAll"]);
        expect(host.querySelector('[aria-label="Activar control interactivo"]').disabled).toBe(true);
    });
});

describe("viewer fullscreen", () => {
    it("opens and closes the exact viewer container", async () => {
        const element = {
            requestFullscreen: jest.fn().mockResolvedValue(undefined),
        };
        const documentObject = {
            fullscreenElement: null,
            exitFullscreen: jest.fn().mockResolvedValue(undefined),
        };

        expect(await toggleViewerFullscreen(element, documentObject)).toBe(true);
        expect(element.requestFullscreen).toHaveBeenCalledTimes(1);

        documentObject.fullscreenElement = element;
        expect(isViewerFullscreen(element, documentObject)).toBe(true);
        expect(await toggleViewerFullscreen(element, documentObject)).toBe(false);
        expect(documentObject.exitFullscreen).toHaveBeenCalledTimes(1);
    });

    it("supports the legacy WebKit names used by embedded Safari viewers", async () => {
        const element = {
            webkitRequestFullScreen: jest.fn(),
        };
        const documentObject = {
            webkitCurrentFullScreenElement: null,
            webkitCancelFullScreen: jest.fn(),
        };

        expect(await toggleViewerFullscreen(element, documentObject)).toBe(true);
        expect(element.webkitRequestFullScreen).toHaveBeenCalledTimes(1);

        documentObject.webkitCurrentFullScreenElement = element;
        expect(isViewerFullscreen(element, documentObject)).toBe(true);
        expect(await toggleViewerFullscreen(element, documentObject)).toBe(false);
        expect(documentObject.webkitCancelFullScreen).toHaveBeenCalledTimes(1);
    });

    it("updates its clear control when native Escape leaves fullscreen", async () => {
        const host = document.createElement("div");
        const consoleError = jest.spyOn(console, "error").mockImplementation(() => {});
        document.body.appendChild(host);
        let fullscreenElement = null;
        Object.defineProperty(document, "fullscreenElement", {
            configurable: true,
            get: () => fullscreenElement,
        });

        await act(async () => {
            ReactDOM.render(<HlsViewer url="" token="" expiresAt="2099-01-01T00:00:00Z" />, host);
        });
        const viewer = host.querySelector('[data-testid="private-viewer"]');
        expect(host.querySelector('[data-testid="viewer-status-live"]').textContent).toContain(
            "Estado del visor"
        );
        viewer.requestFullscreen = jest.fn(() => {
            fullscreenElement = viewer;
            return Promise.resolve();
        });

        await act(async () => {
            host.querySelector('[aria-label="Abrir visor en pantalla completa"]').click();
            await Promise.resolve();
        });
        expect(viewer.requestFullscreen).toHaveBeenCalledTimes(1);
        expect(host.querySelector('[aria-label="Salir de pantalla completa"]')).not.toBeNull();
        expect(host.textContent).toContain("Salir");

        act(() => {
            // Browsers emit fullscreenchange when the user leaves with Escape.
            fullscreenElement = null;
            document.dispatchEvent(new Event("fullscreenchange"));
        });
        expect(host.querySelector('[aria-label="Abrir visor en pantalla completa"]')).not.toBeNull();
        expect(host.textContent).toContain("Pantalla completa");

        act(() => {
            ReactDOM.unmountComponentAtNode(host);
        });
        host.remove();
        delete document.fullscreenElement;
        const unexpectedErrors = consoleError.mock.calls.filter(
            ([message]) => !String(message).includes("unstable_flushDiscreteUpdates")
        );
        consoleError.mockRestore();
        expect(unexpectedErrors).toEqual([]);
    });
});
