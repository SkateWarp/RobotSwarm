import { useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import Hls from "hls.js";
import { Alert, Box, CircularProgress } from "@mui/material";

const MAX_PUBLISHER_RETRIES = 8;

export const configureHlsRequest = (token) => (request) => {
    request.withCredentials = false;
    request.setRequestHeader("Authorization", `Bearer ${token}`);
    request.setRequestHeader("Cache-Control", "no-cache");
};

function HlsViewer({ url, token, expiresAt, onUnavailable }) {
    const videoRef = useRef(null);
    const [error, setError] = useState("");
    const [connected, setConnected] = useState(false);

    useEffect(() => {
        if (!url || !token) return undefined;

        let closed = false;
        let hls;
        let retryTimer;
        let expirationTimer;
        let retryCount = 0;
        let mediaRecoveryUsed = false;
        let fallbackUsed = false;
        const expiration = Date.parse(expiresAt);

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
            setError("The viewer lease has expired. Open a new viewer.");
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

        const scheduleRetry = (start) => {
            destroyPlayer();
            setConnected(false);
            if (leaseExpired()) {
                showExpired();
                return;
            }
            if (retryCount >= MAX_PUBLISHER_RETRIES) {
                if (tryFallback()) return;
                setError("The worker has not published this view yet.");
                return;
            }

            const delay = Math.min(1000 * 2 ** retryCount, 5000);
            retryCount += 1;
            setError("");
            retryTimer = window.setTimeout(start, delay);
        };

        const start = () => {
            if (closed) return;
            if (leaseExpired()) {
                showExpired();
                return;
            }
            if (!Hls.isSupported()) {
                if (tryFallback()) return;
                setError("This browser does not support the private HLS viewer.");
                return;
            }

            setConnected(false);
            setError("");
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

                retryCount = 0;
                setConnected(true);
                setError("");
                videoRef.current.play().catch(() => {});
            });
            activePlayer.on(Hls.Events.ERROR, (_event, details) => {
                if (closed || activePlayer !== hls || !details.fatal) return;

                const responseCode = Number(details.response?.code);
                if (responseCode === 401 || responseCode === 403) {
                    destroyPlayer();
                    setConnected(false);
                    setError("The viewer lease was rejected.");
                    return;
                }

                if (details.type === Hls.ErrorTypes.NETWORK_ERROR) {
                    scheduleRetry(start);
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
                setError("The video stream could not be played.");
            });
            activePlayer.attachMedia(videoRef.current);
        };

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
            destroyPlayer();
        };
    }, [expiresAt, onUnavailable, token, url]);

    return (
        <Box sx={{ position: "relative", width: "100%", height: "100%" }}>
            <video
                ref={videoRef}
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
            {!error && !connected && (
                <CircularProgress size={32} sx={{ position: "absolute", top: 20, right: 20 }} />
            )}
            {error && (
                <Alert severity="warning" sx={{ position: "absolute", left: 16, right: 16, bottom: 16 }}>
                    {error}
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
};

export default HlsViewer;
