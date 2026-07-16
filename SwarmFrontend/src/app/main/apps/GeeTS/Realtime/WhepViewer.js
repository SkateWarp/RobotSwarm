import { useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import { Alert, Box, CircularProgress } from "@mui/material";

const MAX_PUBLISHER_RETRIES = 8;

const authHeaders = (token) => ({
    Authorization: `Bearer ${token}`,
});

const getIceServers = (header) => {
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
    const error = new Error("The worker has not published this view yet.");
    error.retryable = true;
    return error;
};

function WhepViewer({ url, token, expiresAt }) {
    const videoRef = useRef(null);
    const [error, setError] = useState("");
    const [connected, setConnected] = useState(false);

    useEffect(() => {
        if (!url || !token) return undefined;

        let closed = false;
        let connection;
        let retryTimer;
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
            }).catch(() => {});
        };

        const scheduleRetry = (connect) => {
            closeConnection();
            if (leaseExpired()) {
                setError("The viewer lease has expired. Open a new viewer.");
                return;
            }
            if (retryCount >= MAX_PUBLISHER_RETRIES) {
                setError("The worker has not published this view yet.");
                return;
            }

            const delay = Math.min(1000 * 2 ** retryCount, 5000);
            retryCount += 1;
            setConnected(false);
            setError("");
            retryTimer = window.setTimeout(connect, delay);
        };

        const connect = async () => {
            try {
                if (closed) return;
                if (leaseExpired()) {
                    throw new Error("The viewer lease has expired. Open a new viewer.");
                }
                if (!window.RTCPeerConnection) {
                    throw new Error("This browser does not support WebRTC viewing.");
                }

                setError("");
                setConnected(false);
                const optionsResponse = await fetch(url, {
                    method: "OPTIONS",
                    headers: authHeaders(token),
                    signal: controller.signal,
                });
                if (optionsResponse.status === 404) {
                    throw publisherNotReady();
                }
                if (optionsResponse.status === 401 || optionsResponse.status === 403) {
                    throw new Error("The viewer lease was rejected.");
                }
                if (!optionsResponse.ok) {
                    throw new Error("The viewer route could not be reached.");
                }
                if (closed) return;

                connection = new window.RTCPeerConnection({
                    iceServers: getIceServers(optionsResponse.headers.get("Link")),
                });
                const activeConnection = connection;
                connection.addTransceiver("video", { direction: "recvonly" });
                connection.ontrack = (event) => {
                    if (closed || activeConnection !== connection || !videoRef.current) {
                        return;
                    }

                    const stream = event.streams[0] || new window.MediaStream([event.track]);
                    videoRef.current.srcObject = stream;
                    videoRef.current.play().catch(() => {});
                    setConnected(true);
                    setError("");
                };
                connection.onconnectionstatechange = () => {
                    if (
                        !closed &&
                        activeConnection === connection &&
                        connection.connectionState === "failed"
                    ) {
                        setConnected(false);
                        setError("The video connection was lost.");
                    }
                };

                const offer = await connection.createOffer();
                await connection.setLocalDescription(offer);
                await waitForIce(connection);
                if (closed || activeConnection !== connection) return;

                const response = await fetch(url, {
                    method: "POST",
                    headers: {
                        ...authHeaders(token),
                        "Content-Type": "application/sdp",
                    },
                    body: connection.localDescription.sdp,
                    signal: controller.signal,
                });
                if (response.status === 404) {
                    throw publisherNotReady();
                }
                if (response.status === 401 || response.status === 403) {
                    throw new Error("The viewer lease was rejected.");
                }
                if (response.status !== 201) {
                    throw new Error("The video stream could not be opened.");
                }

                const location = response.headers.get("Location");
                if (!location) {
                    throw new Error("The viewer returned an invalid session.");
                }

                sessionUrl = new window.URL(location, url).toString();
                const answer = await response.text();
                if (!closed && activeConnection === connection) {
                    await connection.setRemoteDescription({
                        type: "answer",
                        sdp: answer,
                    });
                }
            } catch (requestError) {
                if (closed || requestError.name === "AbortError") return;

                deleteSession();
                if (requestError.retryable) {
                    scheduleRetry(connect);
                    return;
                }

                closeConnection();
                setConnected(false);
                setError(requestError.message || "The video stream could not be opened.");
            }
        };

        connect();

        return () => {
            closed = true;
            window.clearTimeout(retryTimer);
            controller.abort();
            closeConnection();
            deleteSession();
        };
    }, [expiresAt, token, url]);

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

WhepViewer.propTypes = {
    url: PropTypes.string.isRequired,
    token: PropTypes.string.isRequired,
    expiresAt: PropTypes.string,
};

export default WhepViewer;
