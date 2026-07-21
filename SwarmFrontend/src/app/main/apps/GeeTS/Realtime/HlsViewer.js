import { useCallback, useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import Hls from "hls.js";
import FullscreenExitRoundedIcon from "@mui/icons-material/FullscreenExitRounded";
import FullscreenRoundedIcon from "@mui/icons-material/FullscreenRounded";
import MouseRoundedIcon from "@mui/icons-material/MouseRounded";
import RefreshRoundedIcon from "@mui/icons-material/RefreshRounded";
import { Alert, Box, Button, Chip, CircularProgress, Tooltip, Typography } from "@mui/material";
import { parseViewerTimestamp } from "./viewerTimestamp";

export const VIEWER_STARTUP_BUDGET_MS = 30000;
export const viewerScreenReaderOnlySx = {
    border: 0,
    clip: "rect(0 0 0 0)",
    height: 1,
    margin: -1,
    overflow: "hidden",
    padding: 0,
    position: "absolute",
    whiteSpace: "nowrap",
    width: 1,
};

export const createHlsRetryWindow = (
    now = () => Date.now(),
    budgetMilliseconds = VIEWER_STARTUP_BUDGET_MS
) => {
    let deadline = now() + budgetMilliseconds;
    let playbackStarted = false;
    let recoveryInProgress = false;

    return {
        markPlaybackStarted() {
            playbackStarted = true;
            recoveryInProgress = false;
        },
        beginOrContinueFailure() {
            if (playbackStarted && !recoveryInProgress) {
                deadline = now() + budgetMilliseconds;
                recoveryInProgress = true;
            }
            return Math.max(0, deadline - now());
        },
    };
};

export const isViewerFullscreen = (element, documentObject = document) => {
    const fullscreenElement =
        documentObject.fullscreenElement ||
        documentObject.webkitFullscreenElement ||
        documentObject.webkitCurrentFullScreenElement ||
        null;
    return Boolean(element && fullscreenElement === element);
};

export const toggleViewerFullscreen = async (element, documentObject = document) => {
    if (!element) return null;

    if (isViewerFullscreen(element, documentObject)) {
        const close =
            documentObject.exitFullscreen ||
            documentObject.webkitExitFullscreen ||
            documentObject.webkitCancelFullScreen;
        if (!close) return null;
        await Promise.resolve(close.call(documentObject));
        return false;
    }

    const open =
        element.requestFullscreen || element.webkitRequestFullscreen || element.webkitRequestFullScreen;
    if (!open) return null;
    await Promise.resolve(open.call(element));
    return true;
};

export const hlsNetworkFailure = (statusCode) => {
    if (statusCode === 404) return "pending";
    if (statusCode === 429) return "saturated";
    if (statusCode === 502 || statusCode === 504) return "origin";
    return "network";
};

export const hlsFailureMessage = (failure, timedOut = false) => {
    if (failure === "pending") {
        return timedOut
            ? "La vista fue preparada, pero la lista HLS no apareció después de 30 segundos."
            : "La publicación HLS todavía se está preparando…";
    }
    if (failure === "saturated") {
        return timedOut
            ? "El servicio de video sigue saturado (HTTP 429). Espera unos segundos antes de reintentar."
            : "El servicio de video está saturado (HTTP 429); esperando antes de reintentar…";
    }
    if (failure === "origin") {
        return timedOut
            ? "El origen de video no está disponible (HTTP 502/504). Comprueba el worker y vuelve a intentar."
            : "El origen de video no responde (HTTP 502/504); intentando reconectar…";
    }
    return timedOut
        ? "No fue posible contactar el servicio de video durante 30 segundos."
        : "Reconectando con el servicio de video…";
};

export const configureHlsRequest = (token) => (request) => {
    request.withCredentials = false;
    request.setRequestHeader("Authorization", `Bearer ${token}`);
    request.setRequestHeader("Cache-Control", "no-cache");
};

export const leaseSecondsRemaining = (expiresAt, now = Date.now()) => {
    const expiration = parseViewerTimestamp(expiresAt);
    if (!Number.isFinite(expiration)) return null;
    return Math.max(0, Math.ceil((expiration - now) / 1000));
};

export const formatLeaseCountdown = (seconds) => {
    if (seconds === null) return "Acceso temporal";
    if (seconds <= 0) return "Acceso expirado";

    const minutes = Math.floor(seconds / 60);
    const remainder = String(seconds % 60).padStart(2, "0");
    return `Vence en ${minutes}:${remainder}`;
};

export const useLeaseCountdown = (expiresAt) => {
    const [seconds, setSeconds] = useState(() => leaseSecondsRemaining(expiresAt));

    useEffect(() => {
        const update = () => setSeconds(leaseSecondsRemaining(expiresAt));
        update();
        const timer = window.setInterval(update, 1000);
        return () => window.clearInterval(timer);
    }, [expiresAt]);

    return seconds;
};

export const playbackFps = (frames, elapsedMilliseconds) => {
    if (frames <= 0 || elapsedMilliseconds <= 0) return null;
    return Math.round(((frames * 1000) / elapsedMilliseconds) * 10) / 10;
};

const VIEWER_KEY_CODES = new Set([
    "Backspace",
    "Tab",
    "Enter",
    "Escape",
    "Space",
    "PageUp",
    "PageDown",
    "End",
    "Home",
    "ArrowLeft",
    "ArrowUp",
    "ArrowRight",
    "ArrowDown",
    "Insert",
    "Delete",
    "ShiftLeft",
    "ShiftRight",
    "ControlLeft",
    "ControlRight",
    "AltLeft",
    "AltRight",
    "Minus",
    "Equal",
    "BracketLeft",
    "BracketRight",
    "Backslash",
    "Semicolon",
    "Quote",
    "Backquote",
    "Comma",
    "Period",
    "Slash",
]);

export const isViewerKeyCode = (code) =>
    typeof code === "string" &&
    (/^Key[A-Z]$/.test(code) ||
        /^Digit[0-9]$/.test(code) ||
        /^F(?:[1-9]|1[0-2])$/.test(code) ||
        VIEWER_KEY_CODES.has(code));

export const VIEWER_INPUT_INTERVAL_MS = 10;
export const VIEWER_INPUT_DELIVERY_TIMEOUT_MS = 2000;
export const VIEWER_MAX_PENDING_TRANSITIONS = 64;
export const VIEWER_CONTROL_RELEASE_MARGIN_SECONDS = 5;

const isViewerTransitionInput = (input) =>
    ["pointerDown", "pointerUp", "keyDown", "keyUp", "releaseAll"].includes(input?.type);

const isViewerReleaseInput = (input) => ["pointerUp", "keyUp", "releaseAll"].includes(input?.type);

export const createViewerInputQueue = (
    deliver,
    onFailure,
    timerObject = window,
    now = () => performance.now()
) => {
    let transitions = [];
    let pointerMove = null;
    let wheel = null;
    let nextLowPriority = "pointerMove";
    let timer = null;
    let lastDelivery = null;
    let inFlight = false;
    let inFlightInput = null;
    let releaseAllPending = false;
    let accepting = true;
    let closing = false;
    let failed = false;

    const hasPendingInput = () => releaseAllPending || transitions.length > 0 || pointerMove || wheel;

    const takeNextInput = () => {
        if (releaseAllPending) {
            releaseAllPending = false;
            return { type: "releaseAll" };
        }
        if (transitions.length > 0) return transitions.shift();

        if (pointerMove && wheel) {
            if (nextLowPriority === "pointerMove") {
                const input = pointerMove;
                pointerMove = null;
                nextLowPriority = "wheel";
                return input;
            }
            const input = wheel;
            wheel = null;
            nextLowPriority = "pointerMove";
            return input;
        }
        if (pointerMove) {
            const input = pointerMove;
            pointerMove = null;
            return input;
        }
        if (wheel) {
            const input = wheel;
            wheel = null;
            return input;
        }
        return null;
    };

    const fail = (input, requestRelease = true) => {
        pointerMove = null;
        wheel = null;
        transitions = [];
        if (requestRelease && input?.type !== "releaseAll" && inFlightInput?.type !== "releaseAll") {
            releaseAllPending = true;
        }
        if (!failed) {
            failed = true;
            onFailure?.();
        }
    };

    const failClosedForOverflow = () => {
        fail(null);
        schedule();
    };

    let drain;
    const schedule = () => {
        if (timer !== null || inFlight || !hasPendingInput()) return;
        const elapsed = lastDelivery === null ? VIEWER_INPUT_INTERVAL_MS : now() - lastDelivery;
        const delay = Math.max(0, VIEWER_INPUT_INTERVAL_MS - elapsed);
        timer = timerObject.setTimeout(() => drain(), delay);
    };

    drain = () => {
        timer = null;
        const input = takeNextInput();
        if (!input) return;

        lastDelivery = now();
        inFlight = true;
        inFlightInput = input;
        let settled = false;
        let deliveryTimeout = null;
        const settle = (delivered) => {
            if (settled) return;
            settled = true;
            if (deliveryTimeout !== null) timerObject.clearTimeout?.(deliveryTimeout);
            inFlight = false;
            inFlightInput = null;
            if (!delivered) fail(input);
            if (hasPendingInput()) {
                schedule();
            } else if (closing) {
                accepting = false;
            }
        };

        try {
            const delivery = deliver(input);
            if (delivery && typeof delivery.then === "function") {
                deliveryTimeout = timerObject.setTimeout(
                    () => settle(false),
                    VIEWER_INPUT_DELIVERY_TIMEOUT_MS
                );
                Promise.resolve(delivery).then(
                    () => settle(true),
                    () => settle(false)
                );
            } else {
                settle(true);
            }
        } catch (_deliveryError) {
            settle(false);
        }
    };

    const enqueue = (input) => {
        if (!accepting || !input || (failed && !isViewerReleaseInput(input))) return;

        if (isViewerTransitionInput(input)) {
            if (input.type === "releaseAll") {
                release([{ type: "releaseAll" }]);
                return;
            }
            if (transitions.length >= VIEWER_MAX_PENDING_TRANSITIONS) {
                failClosedForOverflow();
                return;
            }
            transitions.push(input);
        } else if (input.type === "pointerMove") {
            pointerMove = input;
        } else if (input.type === "wheel") {
            wheel = wheel
                ? {
                      ...input,
                      deltaX: Math.max(-1000, Math.min(1000, wheel.deltaX + input.deltaX)),
                      deltaY: Math.max(-1000, Math.min(1000, wheel.deltaY + input.deltaY)),
                  }
                : input;
        }
        schedule();
    };

    const release = (inputs) => {
        pointerMove = null;
        wheel = null;
        const releases = inputs.filter(isViewerReleaseInput);
        if (releases.some((input) => input.type === "releaseAll")) {
            // A global release supersedes pending presses and ordinary releases.
            transitions = [];
            releaseAllPending = inFlightInput?.type !== "releaseAll";
        } else {
            const available = Math.max(0, VIEWER_MAX_PENDING_TRANSITIONS - transitions.length);
            if (releases.length > available) {
                failClosedForOverflow();
                return;
            }
            transitions = [...releases, ...transitions];
        }
        schedule();
    };

    const closeAfterReleases = () => {
        accepting = false;
        closing = true;
        pointerMove = null;
        wheel = null;
        transitions = transitions.filter(isViewerReleaseInput);
        if (transitions.length > 0) schedule();
    };

    const resume = () => {
        if (closing) return false;
        failed = false;
        accepting = true;
        return true;
    };

    return { enqueue, release, closeAfterReleases, resume };
};

export const viewerCoordinates = (video, clientX, clientY) => {
    if (!video || video.videoWidth <= 0 || video.videoHeight <= 0) return null;
    const rectangle = video.getBoundingClientRect();
    if (rectangle.width <= 0 || rectangle.height <= 0) return null;

    const scale = Math.min(rectangle.width / video.videoWidth, rectangle.height / video.videoHeight);
    const renderedWidth = video.videoWidth * scale;
    const renderedHeight = video.videoHeight * scale;
    const left = rectangle.left + (rectangle.width - renderedWidth) / 2;
    const top = rectangle.top + (rectangle.height - renderedHeight) / 2;
    if (clientX < left || clientX > left + renderedWidth || clientY < top || clientY > top + renderedHeight) {
        return null;
    }

    return {
        x: Math.max(0, Math.min(1, (clientX - left) / renderedWidth)),
        y: Math.max(0, Math.min(1, (clientY - top) / renderedHeight)),
    };
};

export const usePlaybackFps = (videoRef, active) => {
    const [fps, setFps] = useState(null);

    useEffect(() => {
        const video = videoRef.current;
        if (!active || !video) {
            setFps(null);
            return undefined;
        }

        let cancelled = false;
        let frameRequest;
        let sampleTimer;

        if (typeof video.requestVideoFrameCallback === "function") {
            let frameCount = 0;
            let sampleStartedAt = null;
            const observeFrame = (now) => {
                if (cancelled) return;
                if (sampleStartedAt === null) sampleStartedAt = now;
                frameCount += 1;
                const elapsed = now - sampleStartedAt;
                if (elapsed >= 1000) {
                    setFps(playbackFps(frameCount, elapsed));
                    frameCount = 0;
                    sampleStartedAt = now;
                }
                frameRequest = video.requestVideoFrameCallback(observeFrame);
            };
            frameRequest = video.requestVideoFrameCallback(observeFrame);
        } else if (typeof video.getVideoPlaybackQuality === "function") {
            let previousFrames = video.getVideoPlaybackQuality().totalVideoFrames;
            let previousTime = performance.now();
            sampleTimer = window.setInterval(() => {
                const now = performance.now();
                const currentFrames = video.getVideoPlaybackQuality().totalVideoFrames;
                setFps(playbackFps(currentFrames - previousFrames, now - previousTime));
                previousFrames = currentFrames;
                previousTime = now;
            }, 1000);
        }

        return () => {
            cancelled = true;
            window.clearInterval(sampleTimer);
            if (frameRequest !== undefined && typeof video.cancelVideoFrameCallback === "function") {
                video.cancelVideoFrameCallback(frameRequest);
            }
        };
    }, [active, videoRef]);

    return fps;
};

function HlsViewer({ url, token, expiresAt, onUnavailable, interactiveAvailable, onInput }) {
    const containerRef = useRef(null);
    const videoRef = useRef(null);
    const [error, setError] = useState("");
    const [fullscreenError, setFullscreenError] = useState("");
    const [connected, setConnected] = useState(false);
    const [retryAttempt, setRetryAttempt] = useState(0);
    const [waitingMessage, setWaitingMessage] = useState("Conectando con la vista privada…");
    const [retryKey, setRetryKey] = useState(0);
    const [isFullscreen, setIsFullscreen] = useState(false);
    const [interactionEnabled, setInteractionEnabled] = useState(false);
    const [interactionError, setInteractionError] = useState("");
    const pressedButtonsRef = useRef(new Set());
    const pressedKeysRef = useRef(new Set());
    const lastPointerMoveRef = useRef(0);
    const lastPointerPositionRef = useRef({ x: 0.5, y: 0.5 });
    const controlWasActiveRef = useRef(false);
    const fullscreenEscapeRef = useRef(false);
    const viewerFullscreenRef = useRef(false);
    const mountedRef = useRef(true);
    const onInputRef = useRef(onInput);
    onInputRef.current = onInput;
    const remainingSeconds = useLeaseCountdown(expiresAt);
    const fps = usePlaybackFps(videoRef, connected);
    const interactionHasTime =
        remainingSeconds === null || remainingSeconds > VIEWER_CONTROL_RELEASE_MARGIN_SECONDS;

    useEffect(
        () => () => {
            mountedRef.current = false;
        },
        []
    );

    const handleInputFailure = useCallback(() => {
        if (!mountedRef.current) return;
        setInteractionEnabled(false);
        setInteractionError("Se perdió el canal de control. Actívalo de nuevo para continuar.");
    }, []);

    const inputQueueRef = useRef(null);
    if (inputQueueRef.current === null) {
        inputQueueRef.current = createViewerInputQueue((input) => {
            const deliver = onInputRef.current;
            if (!deliver) {
                return Promise.reject(new Error("Viewer input is not available."));
            }
            return deliver(input);
        }, handleInputFailure);
    }
    const inputQueue = inputQueueRef.current;

    const releasePressedInputs = useCallback(() => {
        const shouldRelease =
            controlWasActiveRef.current ||
            pressedButtonsRef.current.size > 0 ||
            pressedKeysRef.current.size > 0;
        controlWasActiveRef.current = false;
        pressedButtonsRef.current.clear();
        pressedKeysRef.current.clear();
        if (shouldRelease) inputQueue?.release([{ type: "releaseAll" }]);
    }, [inputQueue]);

    const sendInput = useCallback(
        (input) => {
            if (!interactionEnabled || !interactionHasTime || !inputQueue) return;
            controlWasActiveRef.current = true;
            inputQueue.enqueue(input);
        },
        [inputQueue, interactionEnabled, interactionHasTime]
    );

    useEffect(() => {
        if (connected && interactiveAvailable && onInput) return undefined;
        releasePressedInputs();
        setInteractionEnabled(false);
        return undefined;
    }, [connected, interactiveAvailable, onInput, releasePressedInputs]);

    useEffect(
        () => () => {
            releasePressedInputs();
            inputQueue?.closeAfterReleases();
        },
        [inputQueue, releasePressedInputs]
    );

    useEffect(() => {
        if (interactionEnabled) return;
        releasePressedInputs();
    }, [interactionEnabled, releasePressedInputs]);

    useEffect(() => {
        if (interactionHasTime) return;
        releasePressedInputs();
        setInteractionEnabled(false);
    }, [interactionHasTime, releasePressedInputs]);

    useEffect(() => {
        if (!interactionEnabled) return undefined;
        const stopInteraction = () => {
            releasePressedInputs();
            setInteractionEnabled(false);
        };
        const releaseWhenHidden = () => {
            if (document.visibilityState === "hidden") stopInteraction();
        };
        window.addEventListener("blur", stopInteraction);
        document.addEventListener("visibilitychange", releaseWhenHidden);
        return () => {
            window.removeEventListener("blur", stopInteraction);
            document.removeEventListener("visibilitychange", releaseWhenHidden);
        };
    }, [interactionEnabled, releasePressedInputs]);

    useEffect(() => {
        const updateFullscreenState = () => {
            const active = isViewerFullscreen(containerRef.current);
            const wasActive = viewerFullscreenRef.current;
            viewerFullscreenRef.current = active;
            if (wasActive && !active) releasePressedInputs();
            setIsFullscreen(active);
            if (!active) setFullscreenError("");
        };
        const showFullscreenError = () => {
            setIsFullscreen(isViewerFullscreen(containerRef.current));
            setFullscreenError("No se pudo cambiar el modo de pantalla completa.");
        };

        document.addEventListener("fullscreenchange", updateFullscreenState);
        document.addEventListener("webkitfullscreenchange", updateFullscreenState);
        document.addEventListener("fullscreenerror", showFullscreenError);
        document.addEventListener("webkitfullscreenerror", showFullscreenError);
        return () => {
            document.removeEventListener("fullscreenchange", updateFullscreenState);
            document.removeEventListener("webkitfullscreenchange", updateFullscreenState);
            document.removeEventListener("fullscreenerror", showFullscreenError);
            document.removeEventListener("webkitfullscreenerror", showFullscreenError);
        };
    }, [releasePressedInputs]);

    useEffect(() => {
        if (!url || !token) return undefined;

        let closed = false;
        let hls;
        let retryTimer;
        let expirationTimer;
        let retryCount = 0;
        let mediaRecoveryUsed = false;
        let fallbackUsed = false;
        const expiration = parseViewerTimestamp(expiresAt);
        const retryWindow = createHlsRetryWindow();
        const video = videoRef.current;

        const markLive = () => {
            if (closed || !hls) return;
            retryWindow.markPlaybackStarted();
            retryCount = 0;
            setRetryAttempt(0);
            setWaitingMessage("");
            setError("");
            setConnected(true);
        };
        video?.addEventListener("playing", markLive);

        const leaseExpired = () => Number.isFinite(expiration) && expiration <= Date.now();

        const destroyPlayer = () => {
            if (hls) {
                hls.destroy();
                hls = null;
            }
            if (videoRef.current) {
                videoRef.current.removeAttribute("src");
                videoRef.current.load();
            }
        };

        const showExpired = () => {
            destroyPlayer();
            setConnected(false);
            setError("El acceso al visor expiró. Abre un visor nuevo para continuar.");
        };

        const tryFallback = () => {
            if (!fallbackUsed && onUnavailable) {
                fallbackUsed = true;
                destroyPlayer();
                onUnavailable();
                return true;
            }
            return false;
        };

        const scheduleRetry = (start, failure) => {
            setConnected(false);
            if (leaseExpired()) {
                showExpired();
                return;
            }
            const timeRemaining = retryWindow.beginOrContinueFailure();
            destroyPlayer();
            if (timeRemaining <= 0) {
                if (tryFallback()) return;
                setWaitingMessage("");
                setError(hlsFailureMessage(failure, true));
                return;
            }

            let delay = Math.min(1000 * 2 ** retryCount, 5000);
            if (failure === "saturated") delay = 5000;
            if (failure === "origin") delay = 2500;
            retryCount += 1;
            setRetryAttempt(retryCount);
            setWaitingMessage(hlsFailureMessage(failure));
            setError("");
            retryTimer = window.setTimeout(start, Math.min(delay, timeRemaining));
        };

        const start = () => {
            if (closed) return;
            if (leaseExpired()) {
                showExpired();
                return;
            }
            if (!Hls.isSupported()) {
                if (tryFallback()) return;
                setError("Este navegador no permite reproducir el visor HLS privado.");
                return;
            }

            setConnected(false);
            setError("");
            setWaitingMessage("Conectando con la vista privada…");
            mediaRecoveryUsed = false;
            hls = new Hls({
                lowLatencyMode: true,
                backBufferLength: 0,
                maxBufferLength: 6,
                manifestLoadingMaxRetry: 0,
                xhrSetup: configureHlsRequest(token),
            });
            const activePlayer = hls;

            activePlayer.on(Hls.Events.MEDIA_ATTACHED, () => {
                if (!closed && activePlayer === hls) {
                    activePlayer.loadSource(url);
                }
            });
            activePlayer.on(Hls.Events.MANIFEST_PARSED, () => {
                if (closed || activePlayer !== hls || !videoRef.current) return;

                setWaitingMessage("Iniciando reproducción…");
                videoRef.current.play().catch(() => {});
            });
            activePlayer.on(Hls.Events.ERROR, (_event, details) => {
                if (closed || activePlayer !== hls || !details.fatal) return;

                const responseCode = Number(details.response?.code ?? details.response?.status);
                if (responseCode === 401 || responseCode === 403) {
                    destroyPlayer();
                    setConnected(false);
                    setError("El servidor rechazó el acceso temporal a este visor.");
                    return;
                }

                if (details.type === Hls.ErrorTypes.NETWORK_ERROR) {
                    scheduleRetry(start, hlsNetworkFailure(responseCode));
                    return;
                }

                if (details.type === Hls.ErrorTypes.MEDIA_ERROR && !mediaRecoveryUsed) {
                    mediaRecoveryUsed = true;
                    activePlayer.recoverMediaError();
                    return;
                }

                destroyPlayer();
                setConnected(false);
                if (tryFallback()) return;
                setError("No fue posible reproducir la transmisión de video.");
            });
            activePlayer.attachMedia(videoRef.current);
        };

        setRetryAttempt(0);
        setWaitingMessage("Conectando con la vista privada…");
        setError("");
        if (Number.isFinite(expiration)) {
            expirationTimer = window.setTimeout(
                showExpired,
                Math.min(Math.max(0, expiration - Date.now()), 2147483647)
            );
        }
        start();

        return () => {
            closed = true;
            window.clearTimeout(retryTimer);
            window.clearTimeout(expirationTimer);
            video?.removeEventListener("playing", markLive);
            destroyPlayer();
        };
    }, [expiresAt, onUnavailable, retryKey, token, url]);

    const changeFullscreen = async () => {
        const element = containerRef.current;
        setFullscreenError("");
        try {
            const active = await toggleViewerFullscreen(element);
            if (active === null) {
                setFullscreenError("La pantalla completa no está disponible en este navegador.");
                return;
            }
            setIsFullscreen(active);
        } catch (_requestError) {
            setFullscreenError(
                isFullscreen
                    ? "No se pudo salir de la pantalla completa."
                    : "No se pudo abrir el visor en pantalla completa."
            );
        }
    };

    const changeInteraction = () => {
        if (!connected || !interactiveAvailable || !onInput || !interactionHasTime) return;
        if (interactionEnabled) {
            releasePressedInputs();
            setInteractionEnabled(false);
            return;
        }
        if (!inputQueue.resume()) return;
        setInteractionError("");
        controlWasActiveRef.current = true;
        setInteractionEnabled(true);
        window.setTimeout(() => videoRef.current?.focus(), 0);
    };

    const pointerInput = (type) => (event) => {
        if (!interactionEnabled) return;
        let coordinates = viewerCoordinates(videoRef.current, event.clientX, event.clientY);
        if (type === "pointerUp" && !coordinates && pressedButtonsRef.current.has(event.button)) {
            coordinates = lastPointerPositionRef.current;
        }
        if (!coordinates) return;
        if ((type === "pointerDown" || type === "pointerUp") && ![0, 1, 2].includes(event.button)) {
            return;
        }
        event.preventDefault();
        lastPointerPositionRef.current = coordinates;
        videoRef.current?.focus();
        if (type === "pointerMove") {
            const now = performance.now();
            if (now - lastPointerMoveRef.current < 33) return;
            lastPointerMoveRef.current = now;
        }
        if (type === "pointerDown") {
            pressedButtonsRef.current.add(event.button);
            event.currentTarget.setPointerCapture?.(event.pointerId);
        } else if (type === "pointerUp") {
            pressedButtonsRef.current.delete(event.button);
            event.currentTarget.releasePointerCapture?.(event.pointerId);
        }
        sendInput({
            type,
            ...coordinates,
            ...(type === "pointerDown" || type === "pointerUp" ? { button: event.button } : {}),
        });
    };

    const wheelInput = (event) => {
        if (!interactionEnabled) return;
        const coordinates = viewerCoordinates(videoRef.current, event.clientX, event.clientY);
        if (!coordinates) return;
        event.preventDefault();
        sendInput({
            type: "wheel",
            ...coordinates,
            deltaX: Math.max(-1000, Math.min(1000, event.deltaX)),
            deltaY: Math.max(-1000, Math.min(1000, event.deltaY)),
        });
    };

    const keyInput = (type) => (event) => {
        if (!interactionEnabled || !isViewerKeyCode(event.code)) return;
        const leavesViewerFullscreen = event.code === "Escape" && isViewerFullscreen(containerRef.current);
        const finishesFullscreenEscape =
            event.code === "Escape" && type === "keyUp" && fullscreenEscapeRef.current;
        const repeatsFullscreenEscape =
            event.code === "Escape"
            && type === "keyDown"
            && event.repeat
            && fullscreenEscapeRef.current;
        if (repeatsFullscreenEscape) {
            event.preventDefault();
            return;
        }
        if (leavesViewerFullscreen || finishesFullscreenEscape) {
            fullscreenEscapeRef.current = type === "keyDown";
            if (type === "keyDown") releasePressedInputs();
            else event.preventDefault();
            return;
        }
        if (event.code === "Escape" && type === "keyDown") fullscreenEscapeRef.current = false;
        event.preventDefault();
        if (type === "keyDown") {
            if (event.repeat) return;
            pressedKeysRef.current.add(event.code);
        } else {
            pressedKeysRef.current.delete(event.code);
        }
        sendInput({ type, code: event.code });
    };

    let status = "Conectando";
    let statusColor = "info";
    if (connected) {
        status = "En vivo";
        statusColor = "success";
    }
    if (error) {
        status = "Error";
        statusColor = "error";
    }
    const countdown = formatLeaseCountdown(remainingSeconds);
    const compactCountdown = remainingSeconds === 0 ? "Expirado" : countdown.replace("Vence en ", "");
    const fpsLabel = fps === null ? "Video FPS —" : `Video ${fps.toFixed(1)} FPS`;
    const compactFpsLabel = fps === null ? "FPS —" : `${fps.toFixed(1)} FPS`;
    const interactionLabel = interactionEnabled
        ? "Desactivar control interactivo"
        : "Activar control interactivo";
    const fullscreenLabel = isFullscreen
        ? "Salir de pantalla completa"
        : "Abrir visor en pantalla completa";
    let alertSeverity = "info";
    if (error) alertSeverity = "warning";
    if (interactionError) alertSeverity = "error";

    return (
        <Box
            ref={containerRef}
            data-testid="private-viewer"
            sx={{
                position: "relative",
                width: "100%",
                height: "100%",
                bgcolor: "#111827",
                overflow: "hidden",
                "&:fullscreen, &:-webkit-full-screen": {
                    width: "100vw",
                    height: "100vh",
                    maxWidth: "none",
                    maxHeight: "none",
                    borderRadius: 0,
                },
            }}
        >
            <video
                ref={videoRef}
                data-testid="viewer-video"
                aria-label="Transmisión privada de la simulación"
                autoPlay
                muted
                playsInline
                tabIndex={interactionEnabled ? 0 : -1}
                onPointerMove={pointerInput("pointerMove")}
                onPointerDown={pointerInput("pointerDown")}
                onPointerUp={pointerInput("pointerUp")}
                onLostPointerCapture={() => {
                    if (pressedButtonsRef.current.size > 0) releasePressedInputs();
                }}
                onPointerCancel={() => {
                    releasePressedInputs();
                    setInteractionEnabled(false);
                }}
                onWheel={wheelInput}
                onKeyDown={keyInput("keyDown")}
                onKeyUp={keyInput("keyUp")}
                onBlur={releasePressedInputs}
                onContextMenu={(event) => interactionEnabled && event.preventDefault()}
                style={{
                    width: "100%",
                    height: "100%",
                    objectFit: "contain",
                    background: "#111827",
                    cursor: interactionEnabled ? "crosshair" : "default",
                    outline: interactionEnabled ? "2px solid #22d3ee" : "none",
                    outlineOffset: -2,
                }}
            />

            <Box
                sx={{
                    position: "absolute",
                    top: { xs: 6, sm: 12 },
                    left: { xs: 6, sm: 12 },
                    right: { xs: 6, sm: 12 },
                    display: "flex",
                    alignItems: "center",
                    gap: { xs: 0.5, sm: 1 },
                    flexWrap: { xs: "nowrap", sm: "wrap" },
                    pointerEvents: "none",
                }}
            >
                <Box
                    component="span"
                    data-testid="viewer-status-live"
                    aria-live="polite"
                    sx={viewerScreenReaderOnlySx}
                >
                    Estado del visor: {status}
                </Box>
                <Chip
                    data-testid="viewer-status"
                    aria-hidden="true"
                    label={status}
                    color={statusColor}
                    size="small"
                    sx={{ display: { xs: "none", sm: "inline-flex" }, fontWeight: 700 }}
                />
                <Chip
                    data-testid="viewer-lease-countdown"
                    aria-label={countdown}
                    label={
                        <>
                            <Box component="span" sx={{ display: { xs: "inline", sm: "none" } }}>
                                {compactCountdown}
                            </Box>
                            <Box component="span" sx={{ display: { xs: "none", sm: "inline" } }}>
                                {countdown}
                            </Box>
                        </>
                    }
                    color={remainingSeconds === 0 ? "error" : "default"}
                    size="small"
                    sx={{ bgcolor: "rgba(255,255,255,.9)" }}
                />
                <Chip
                    data-testid="viewer-fps"
                    aria-label={fpsLabel}
                    label={
                        <>
                            <Box component="span" sx={{ display: { xs: "inline", sm: "none" } }}>
                                {compactFpsLabel}
                            </Box>
                            <Box component="span" sx={{ display: { xs: "none", sm: "inline" } }}>
                                {fpsLabel}
                            </Box>
                        </>
                    }
                    size="small"
                    sx={{ bgcolor: "rgba(255,255,255,.9)" }}
                />
                <Box sx={{ flex: 1 }} />
                <Tooltip describeChild title={interactionLabel}>
                    <Box component="span" sx={{ display: "inline-flex", pointerEvents: "auto" }}>
                        <Button
                            aria-label={interactionLabel}
                            aria-pressed={interactionEnabled}
                            size="small"
                            variant="contained"
                            startIcon={<MouseRoundedIcon />}
                            disabled={!connected || !interactiveAvailable || !interactionHasTime}
                            onClick={changeInteraction}
                            sx={{
                                color: "common.white",
                                bgcolor: interactionEnabled ? "success.main" : "rgba(17,24,39,.78)",
                                textTransform: "none",
                                whiteSpace: "nowrap",
                                minWidth: { xs: 36, sm: "auto" },
                                px: { xs: 1, sm: 1.5 },
                                "& .MuiButton-startIcon": { mr: { xs: 0, sm: 0.75 } },
                                "&:hover": {
                                    bgcolor: interactionEnabled
                                        ? "success.dark"
                                        : "rgba(17,24,39,.96)",
                                },
                            }}
                        >
                            <Box component="span" sx={{ display: { xs: "none", sm: "inline" } }}>
                                {interactionEnabled ? "Control activo" : "Interactuar"}
                            </Box>
                        </Button>
                    </Box>
                </Tooltip>
                <Tooltip describeChild title={fullscreenLabel}>
                    <Box component="span" sx={{ display: "inline-flex", pointerEvents: "auto" }}>
                        <Button
                            aria-label={fullscreenLabel}
                            aria-pressed={isFullscreen}
                            size="small"
                            variant="contained"
                            startIcon={
                                isFullscreen ? <FullscreenExitRoundedIcon /> : <FullscreenRoundedIcon />
                            }
                            onClick={changeFullscreen}
                            sx={{
                                color: "common.white",
                                bgcolor: "rgba(17,24,39,.78)",
                                textTransform: "none",
                                whiteSpace: "nowrap",
                                minWidth: { xs: 36, sm: "auto" },
                                px: { xs: 1, sm: 1.5 },
                                "& .MuiButton-startIcon": { mr: { xs: 0, sm: 0.75 } },
                                "&:hover": { bgcolor: "rgba(17,24,39,.96)" },
                            }}
                        >
                            <Box component="span" sx={{ display: { xs: "none", sm: "inline" } }}>
                                {isFullscreen ? "Salir" : "Pantalla completa"}
                            </Box>
                        </Button>
                    </Box>
                </Tooltip>
            </Box>

            {!error && !connected && (
                <Box
                    aria-live="polite"
                    sx={{
                        position: "absolute",
                        inset: 0,
                        display: "flex",
                        flexDirection: "column",
                        alignItems: "center",
                        justifyContent: "center",
                        gap: 1.5,
                        color: "grey.100",
                        pointerEvents: "none",
                    }}
                >
                    <CircularProgress size={34} color="inherit" />
                    <Typography variant="body2">{waitingMessage}</Typography>
                    {retryAttempt > 0 && (
                        <Typography variant="caption" color="grey.400">
                            Reintento {retryAttempt}
                        </Typography>
                    )}
                </Box>
            )}

            {interactionEnabled && !error && (
                <Typography
                    variant="caption"
                    sx={{
                        position: "absolute",
                        left: 12,
                        bottom: 12,
                        color: "common.white",
                        bgcolor: "rgba(17,24,39,.78)",
                        px: 1.5,
                        py: 0.75,
                        borderRadius: 1,
                        pointerEvents: "none",
                    }}
                >
                    Control remoto activo · clic, arrastre, rueda y teclado · Esc sale de pantalla completa
                </Typography>
            )}

            {(error || fullscreenError || interactionError) && (
                <Alert
                    severity={alertSeverity}
                    aria-live="assertive"
                    action={
                        error && remainingSeconds !== 0 ? (
                            <Button
                                color="inherit"
                                size="small"
                                startIcon={<RefreshRoundedIcon />}
                                onClick={() => setRetryKey((current) => current + 1)}
                            >
                                Reintentar visor
                            </Button>
                        ) : null
                    }
                    sx={{ position: "absolute", left: 12, right: 12, bottom: 12 }}
                >
                    {error || interactionError || fullscreenError}
                </Alert>
            )}
        </Box>
    );
}

HlsViewer.propTypes = {
    url: PropTypes.string.isRequired,
    token: PropTypes.string.isRequired,
    expiresAt: PropTypes.string,
    onUnavailable: PropTypes.func,
    interactiveAvailable: PropTypes.bool,
    onInput: PropTypes.func,
};

HlsViewer.defaultProps = {
    interactiveAvailable: false,
    onInput: undefined,
};

export default HlsViewer;
