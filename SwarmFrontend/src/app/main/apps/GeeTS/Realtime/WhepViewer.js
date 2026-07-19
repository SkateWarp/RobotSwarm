import { useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import FullscreenExitRoundedIcon from "@mui/icons-material/FullscreenExitRounded";
import FullscreenRoundedIcon from "@mui/icons-material/FullscreenRounded";
import RefreshRoundedIcon from "@mui/icons-material/RefreshRounded";
import { Alert, Box, Button, Chip, CircularProgress, Typography } from "@mui/material";
import {
    formatLeaseCountdown,
    isViewerFullscreen,
    toggleViewerFullscreen,
    useLeaseCountdown,
    usePlaybackFps,
} from "./HlsViewer";

const MAX_PUBLISHER_RETRIES = 8;

const authHeaders = (token) => ({
    Authorization: `Bearer ${token}`,
});

export const getIceServers = (header) => {
    if (!header) return [];

    return header
        .split(",")
        .map((entry) => {
            const url = entry.match(/<([^>]+)>/)?.[1];
            const username = entry.match(/username="([^"]+)"/)?.[1];
            const credential = entry.match(/credential="([^"]+)"/)?.[1];
            return {
                urls: [url],
                ...(username ? { username } : {}),
                ...(credential ? { credential } : {}),
            };
        })
        .filter((server) => server.urls[0]);
};

const waitForIce = (connection) =>
    new Promise((resolve) => {
        if (connection.iceGatheringState === "complete") {
            resolve();
            return;
        }

        let timeout;
        const finish = () => {
            window.clearTimeout(timeout);
            connection.removeEventListener("icegatheringstatechange", onChange);
            resolve();
        };
        const onChange = () => {
            if (connection.iceGatheringState === "complete") {
                finish();
            }
        };

        timeout = window.setTimeout(finish, 4000);
        connection.addEventListener("icegatheringstatechange", onChange);
    });

const publisherNotReady = () => {
    const error = new Error("El worker todavía no ha publicado esta vista.");
    error.retryable = true;
    return error;
};

const sameOriginSessionUrl = (location, endpoint) => {
    const sessionUrl = new window.URL(location, endpoint);
    if (sessionUrl.origin !== new window.URL(endpoint).origin) {
        throw new Error("El visor devolvió una sesión WebRTC no válida.");
    }
    return sessionUrl.toString();
};

function WhepViewer({ url, token, expiresAt }) {
    const containerRef = useRef(null);
    const videoRef = useRef(null);
    const [error, setError] = useState("");
    const [fullscreenError, setFullscreenError] = useState("");
    const [connected, setConnected] = useState(false);
    const [retryAttempt, setRetryAttempt] = useState(0);
    const [retryKey, setRetryKey] = useState(0);
    const [isFullscreen, setIsFullscreen] = useState(false);
    const remainingSeconds = useLeaseCountdown(expiresAt);
    const fps = usePlaybackFps(videoRef, connected);

    useEffect(() => {
        const updateFullscreenState = () => {
            const active = isViewerFullscreen(containerRef.current);
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
    }, []);

    useEffect(() => {
        const video = videoRef.current;
        if (!video) return undefined;
        const markLive = () => setConnected(true);
        video.addEventListener("playing", markLive);
        return () => video.removeEventListener("playing", markLive);
    }, []);

    useEffect(() => {
        if (!url || !token) return undefined;

        let closed = false;
        let leaseEnded = false;
        let connection;
        let retryTimer;
        let expirationTimer;
        let retryCount = 0;
        let sessionUrl;
        const controller = new AbortController();
        const expiration = Date.parse(expiresAt);

        const leaseExpired = () => Number.isFinite(expiration) && expiration <= Date.now();

        const clearVideo = () => {
            if (videoRef.current) {
                videoRef.current.srcObject = null;
            }
        };

        const closeConnection = () => {
            if (connection) {
                connection.ontrack = null;
                connection.onconnectionstatechange = null;
                connection.close();
                connection = null;
            }
            clearVideo();
        };

        const deleteSession = () => {
            if (!sessionUrl) return;
            const currentSessionUrl = sessionUrl;
            sessionUrl = null;
            fetch(currentSessionUrl, {
                method: "DELETE",
                headers: authHeaders(token),
                credentials: "omit",
                keepalive: true,
            }).catch(() => {});
        };

        const stopCurrentSession = () => {
            deleteSession();
            closeConnection();
        };

        const showExpired = () => {
            if (leaseEnded || closed) return;
            leaseEnded = true;
            window.clearTimeout(retryTimer);
            controller.abort();
            stopCurrentSession();
            setConnected(false);
            setError("El acceso al visor expiró. Abre un visor nuevo para continuar.");
        };

        const scheduleRetry = (connect) => {
            stopCurrentSession();
            if (leaseExpired()) {
                showExpired();
                return;
            }
            if (retryCount >= MAX_PUBLISHER_RETRIES) {
                setError(
                    "El worker todavía no ha publicado esta vista. Puedes reintentar sin crear otra sesión."
                );
                return;
            }

            const delay = Math.min(1000 * 2 ** retryCount, 5000);
            retryCount += 1;
            setRetryAttempt(retryCount);
            setConnected(false);
            setError("");
            retryTimer = window.setTimeout(connect, delay);
        };

        const connect = async () => {
            try {
                if (closed || leaseEnded) return;
                if (leaseExpired()) {
                    showExpired();
                    return;
                }
                if (!window.RTCPeerConnection) {
                    throw new Error("Este navegador no permite abrir el visor WebRTC.");
                }

                setError("");
                setConnected(false);
                const optionsResponse = await fetch(url, {
                    method: "OPTIONS",
                    headers: authHeaders(token),
                    credentials: "omit",
                    signal: controller.signal,
                });
                if (optionsResponse.status === 404) {
                    throw publisherNotReady();
                }
                if (optionsResponse.status === 401 || optionsResponse.status === 403) {
                    throw new Error("El servidor rechazó el acceso temporal a este visor.");
                }
                if (!optionsResponse.ok) {
                    throw new Error("No fue posible contactar la ruta privada del visor.");
                }
                if (closed || leaseEnded) return;

                connection = new window.RTCPeerConnection({
                    iceServers: getIceServers(optionsResponse.headers.get("Link")),
                });
                const activeConnection = connection;
                connection.addTransceiver("video", { direction: "recvonly" });
                connection.ontrack = (event) => {
                    if (closed || leaseEnded || activeConnection !== connection || !videoRef.current) {
                        return;
                    }

                    const stream = event.streams[0] || new window.MediaStream([event.track]);
                    videoRef.current.srcObject = stream;
                    videoRef.current.play().catch(() => {});
                };
                connection.onconnectionstatechange = () => {
                    if (
                        !closed &&
                        !leaseEnded &&
                        activeConnection === connection &&
                        connection.connectionState === "failed"
                    ) {
                        scheduleRetry(connect);
                    }
                };

                const offer = await connection.createOffer();
                await connection.setLocalDescription(offer);
                await waitForIce(connection);
                if (closed || leaseEnded || activeConnection !== connection) return;

                const response = await fetch(url, {
                    method: "POST",
                    headers: {
                        ...authHeaders(token),
                        "Content-Type": "application/sdp",
                    },
                    credentials: "omit",
                    body: connection.localDescription.sdp,
                    signal: controller.signal,
                });
                if (response.status === 404) {
                    throw publisherNotReady();
                }
                if (response.status === 401 || response.status === 403) {
                    throw new Error("El servidor rechazó el acceso temporal a este visor.");
                }
                if (response.status !== 201) {
                    throw new Error("No fue posible abrir la transmisión de video.");
                }

                const location = response.headers.get("Location");
                if (!location) {
                    throw new Error("El visor devolvió una sesión WebRTC no válida.");
                }

                sessionUrl = sameOriginSessionUrl(location, url);
                const answer = await response.text();
                if (!closed && !leaseEnded && activeConnection === connection) {
                    await connection.setRemoteDescription({
                        type: "answer",
                        sdp: answer,
                    });
                    retryCount = 0;
                    setRetryAttempt(0);
                }
            } catch (requestError) {
                if (closed || leaseEnded || requestError.name === "AbortError") return;

                if (requestError.retryable) {
                    scheduleRetry(connect);
                    return;
                }

                stopCurrentSession();
                setConnected(false);
                setError(requestError.message || "No fue posible abrir la transmisión de video.");
            }
        };

        setRetryAttempt(0);
        setError("");
        if (leaseExpired()) {
            showExpired();
        } else {
            if (Number.isFinite(expiration)) {
                expirationTimer = window.setTimeout(
                    showExpired,
                    Math.min(Math.max(0, expiration - Date.now()), 2147483647)
                );
            }
            connect();
        }

        return () => {
            closed = true;
            window.clearTimeout(retryTimer);
            window.clearTimeout(expirationTimer);
            controller.abort();
            stopCurrentSession();
        };
    }, [expiresAt, retryKey, token, url]);

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
                style={{
                    width: "100%",
                    height: "100%",
                    objectFit: "contain",
                    background: "#111827",
                }}
            />

            <Box
                sx={{
                    position: "absolute",
                    top: 12,
                    left: 12,
                    right: 12,
                    display: "flex",
                    alignItems: "center",
                    gap: 1,
                    flexWrap: "wrap",
                    pointerEvents: "none",
                }}
            >
                <Chip
                    data-testid="viewer-status"
                    aria-live="polite"
                    label={status}
                    color={statusColor}
                    size="small"
                    sx={{ fontWeight: 700 }}
                />
                <Chip
                    data-testid="viewer-lease-countdown"
                    label={countdown}
                    color={remainingSeconds === 0 ? "error" : "default"}
                    size="small"
                    sx={{ bgcolor: "rgba(255,255,255,.9)" }}
                />
                <Chip
                    data-testid="viewer-fps"
                    label={fps === null ? "Video FPS —" : `Video ${fps.toFixed(1)} FPS`}
                    size="small"
                    sx={{ bgcolor: "rgba(255,255,255,.9)" }}
                />
                <Box sx={{ flex: 1 }} />
                <Button
                    aria-label={
                        isFullscreen ? "Salir de pantalla completa" : "Abrir visor en pantalla completa"
                    }
                    aria-pressed={isFullscreen}
                    size="small"
                    variant="contained"
                    startIcon={isFullscreen ? <FullscreenExitRoundedIcon /> : <FullscreenRoundedIcon />}
                    onClick={changeFullscreen}
                    sx={{
                        color: "common.white",
                        bgcolor: "rgba(17,24,39,.78)",
                        pointerEvents: "auto",
                        textTransform: "none",
                        whiteSpace: "nowrap",
                        "&:hover": { bgcolor: "rgba(17,24,39,.96)" },
                    }}
                >
                    {isFullscreen ? "Salir" : "Pantalla completa"}
                </Button>
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
                    <Typography variant="body2">
                        {retryAttempt > 0
                            ? `Esperando la publicación del worker · intento ${retryAttempt}`
                            : "Negociando la transmisión privada…"}
                    </Typography>
                </Box>
            )}

            {connected && !error && (
                <Typography
                    data-testid="whep-fallback-note"
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
                    Modo WebRTC de respaldo · solo video; la interacción requiere la ruta HLS
                </Typography>
            )}

            {(error || fullscreenError) && (
                <Alert
                    severity={error ? "warning" : "info"}
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
                    {error || fullscreenError}
                </Alert>
            )}
        </Box>
    );
}

WhepViewer.propTypes = {
    url: PropTypes.string.isRequired,
    token: PropTypes.string.isRequired,
    expiresAt: PropTypes.string,
};

export default WhepViewer;
