# Scene viewer publisher

`deploy/gpu-worker/robotswarm-viewer-publisher` implements interactive
protocol 2 of the SwarmWorker external viewer contract. It exposes `Scene`
only. The TurtleBot3 Burger image does not yet contain a supported
robot-camera sensor, so the helper deliberately rejects `RobotCamera`.

The GPU-worker workflow syntax-checks and tests this file, packages it with
mode `0755`, and gives each versioned release an absolute
`Worker__Viewer__PublisherExecutable` path. The owner-only identity file keeps
viewer publishing disabled by default.

For each viewer lease, the helper:

1. takes an advisory lock on an unused private X display and creates a unique
   MIT-MAGIC-COOKIE Xauthority file;
2. starts Xvfb with its authenticated local/abstract Unix transport and
   explicitly disables X11 TCP;
3. read-only inspects the assigned session container and its sole internal
   network, binding their worker/session labels to the IDs supplied by the
   worker and cross-checking the exact private IPv4 endpoint;
4. waits for Gazebo on that exact `<container-ip>:11345`, then runs the host's
   Gazebo Classic 11 `gzclient` in a minimal unprivileged `bwrap` mount
   namespace; no viewer sidecar is created and the simulation is untouched;
5. selects the one matching Gazebo window, moves and resizes it to `0,0` and
   the full private-display size, focuses it, and verifies that geometry;
6. captures the display with FFmpeg and publishes the canonical session path
   to MediaMTX through a credential-hiding loopback RTSP proxy; and
7. prints `READY` only after the GUI plugin binds its report to that gzclient,
   proves the requested D3D GPU adapter at at least 45 FPS under the 50 FPS
   cap, and the proxy observes sustained RTP video traffic.

The short-lived, publish-only lease token is sent as the first bounded line on
the helper's standard input. Protocol 2 keeps that private pipe open for
bounded JSONL pointer, wheel, and keyboard events for the same session and
lease; the helper injects them only into its private display through XTest.
The probe receives closed, empty input. The token is never placed in the
helper or child environments, process arguments, or logs.

FFmpeg publishes to an ephemeral loopback RTSP proxy without credentials in
its output URL. The helper-owned proxy accepts one local publisher, rejects
requests outside the lease's canonical path, and injects MediaMTX's `token`
query into upstream RTSP requests in memory. This preserves per-lease
authorization without exposing the token in `/proc/<pid>/cmdline`. Runtime
files and captured child logs live in a mode-0700 per-user directory and are
removed when the lease stops.

The backend generates this token separately from the browser's read token and
stores only its SHA-256 hash on the viewer lease. The raw token exists only in
the queued `SetViewerSource` payload, worker memory, the short-lived stdin
pipe, and the helper proxy's memory. It expires and is revoked with that exact
lease and stream path. The publisher probe is deliberately credential-free;
the worker enrollment credential is never exposed to the helper or MediaMTX.
Only the non-secret enrolled worker UUID is passed as a separate argument so
the helper can bind Docker ownership labels to the invoking worker.

The worker requests a normal shutdown with `SIGTERM` on Linux and macOS. The
helper handles that signal and tears down the complete per-session pipeline.
`Worker__Viewer__StopTimeoutSeconds` controls the bounded grace period (five
seconds by default); after it expires, the worker force-kills the process tree.
The worker also watches a publisher after `READY`. It restarts an unexpected
exit for the same lease at most three times, after delays of one, two, and four
seconds. Replacement, session stop, service shutdown, and lease expiry cancel
pending recovery. Exhausting the budget leaves that publisher stopped instead
of creating an unbounded restart loop.

Each publishing helper creates a private POSIX process group before launching
Xvfb, the `bwrap` monitor, and FFmpeg. Bubblewrap uses `--die-with-parent`; the
worker also force-kills the complete process tree and reaps the helper's group
before replacement. Recovery fails closed if cleanup cannot be verified. A
mode-0600 per-lease lock prevents concurrent ownership, while a same-UID
mode-0700 runtime directory can be reclaimed after a hard kill; symlinks,
unexpected owners, and permissive stale directories are rejected.

## Host requirements

Install these commands and libraries on the WSL GPU worker before running a
real smoke test:

- Docker with read access to worker session/network metadata;
- host Gazebo Classic 11 (`gzclient`) matching the immutable ROS image;
- `Xvfb` from `xvfb`, `xwininfo` and `xprop` from `x11-utils`, `bubblewrap`,
  and the X11 and XTest runtime libraries;
- FFmpeg with `h264_nvenc` or `libx264`; and
- WSL GPU support through `/dev/dxg` and Mesa D3D12.

The versioned worker release includes the GUI measurement plugin,
`turtlebot3_description`, TurtleBot Gazebo models, and RobotSwarm models
copied from the exact immutable session image. Gzserver publishes absolute
TurtleBot mesh paths, so `bwrap` overlays the packaged ROS share read-only at
`/opt/ros/noetic/share` for gzclient without modifying host `/opt`. The
sandbox exposes only the host runtime roots it needs; it does not bind `/`, a
user home, `/mnt`, or the Docker socket.

This is filesystem/process isolation, not network isolation. Bubblewrap uses
`--share-net` because an unprivileged private network namespace cannot route
directly to the validated Docker bridge. Host gzclient therefore inherits the
worker user's network egress. The design trusts the versioned host Gazebo 11
binary, packaged plugin, and immutable session image; validating the exact
container address constrains `ROS_MASTER_URI` and `GAZEBO_MASTER_URI`. The
helper also binds the unique Docker IPAM gateway locally and exports it as
both `GZ_IP` and Gazebo Classic 11's effective `GAZEBO_IP`, so the internal
container can return Gazebo Transport traffic to gzclient. These checks are
not a firewall against a compromised client.

FFmpeg automatically prefers `h264_nvenc` and performs a one-frame encoder
initialization test with the selected runtime preset, tuning, rate-control,
pixel format, frame size, and frame rate during the helper probe. Set
`ROBOTSWARM_VIEWER_ENCODER=h264_nvenc` to fail closed instead of accepting the
CPU fallback, or `libx264` to select the tested software encoder explicitly.
The deployment persists this identity setting, defaulting to `auto`. The
default scene size is 1280x720 at 30 FPS and 4 Mbit/s.

The default display transport is Unix. WSLg's nested read-only
`/tmp/.X11-unix` mount can prevent Xvfb from creating the filesystem socket
and produce a warning, but Xorg's Linux abstract `local` endpoint still works
with `DISPLAY=:N`. The helper starts Xvfb with `-nolisten tcp -listen local`,
authenticates the endpoint with the lease cookie, checks abstract-socket
collisions, and rejects a lease if port `6000+N` is occupied. `PrivateTmp=true`
therefore stays enabled and no X11 network listener is opened by default.
TCP display transport is intentionally rejected; the MIT cookie is not used
as a substitute for preventing a network listener.

Optional settings are local host configuration, never viewer-command values:

```bash
ROBOTSWARM_VIEWER_DISPLAY_BACKEND=xvfb
ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=unix
ROBOTSWARM_VIEWER_DISPLAY_SERVER=/usr/bin/Xvfb
ROBOTSWARM_VIEWER_GZCLIENT=/usr/bin/gzclient
ROBOTSWARM_VIEWER_SANDBOX=/usr/bin/bwrap
ROBOTSWARM_VIEWER_ENCODER=auto
ROBOTSWARM_VIEWER_RENDER_RATE=50
ROBOTSWARM_VIEWER_MIN_RENDER_RATE=45
ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME=NVIDIA
ROBOTSWARM_VIEWER_WIDTH=1280
ROBOTSWARM_VIEWER_HEIGHT=720
ROBOTSWARM_VIEWER_FRAME_RATE=30
ROBOTSWARM_VIEWER_BIT_RATE=4M
```

The render probe gets a 5-second warmup before measuring the scene. The helper's
default startup budget is 45 seconds, while
`Worker__Viewer__StartupTimeoutSeconds` defaults to 50 seconds. The extra five
seconds are normal-startup headroom, not a guaranteed error-reporting window:
the helper begins its own deadline after initial parsing and dependency checks,
and stopping child processes can consume additional time. Raise both startup
values together if the pinned image needs longer to create its Gazebo window.
`READY` is never emitted after the helper's own budget expires; the worker may
still report a generic timeout if it has to terminate a slow failure path.

The deployment already installs the helper and points
`Worker__Viewer__PublisherExecutable` at its absolute, versioned path. Setting
`Worker__Viewer__Enabled=true` in the identity file makes deployment require a
credential-free RTSP base URL, host dependencies, packaged assets/plugin, and
a successfully initialized H.264 encoder. The deployment passes the transport,
50 FPS cap, 45 FPS minimum, and GPU adapter to the same probe and preserves
them in the release environment. The helper prefers NVENC and can fall back to
`libx264`; keep the backend's worker-publishing gate off for general use until
RTSP ingest, interactive browser delivery, lease expiry, and simultaneous-session
isolation all pass. It may be enabled temporarily, together with the backend
proxy gate, inside a supervised acceptance window.

## Primary authenticated low-latency HLS path

The production stack can deliver the same private RTSP stream without opening
a public MediaMTX port. MediaMTX creates low-latency HLS on port 8888 inside
the Compose network. The backend proxies only canonical session playlists and
MP4 parts below `https://robot.zerav.la/api/viewer/hls`; the browser never
selects the upstream host. Every request carries the short-lived read lease in
the `Authorization: Bearer` header. The backend validates that lease for the
exact session and source path, then replaces it with a separate internal CDN
credential on the private MediaMTX hop. MediaMTX never receives the user's
read token.

`Viewer:LeaseMinutes` vale 15 minutos por defecto en producción y el backend
lo limita al intervalo de 1–30 minutos. Esta ventana cubre las tareas ROS
aceptadas de mayor duración sin convertir el stream en permanente. Crear un
lease sustituto, cerrar el visor, detener la sesión o alcanzar la caducidad
sigue revocando el acceso; el frontend no puede extender el token de lectura
por el solo hecho de renovar su grant interactivo de SignalR.

The proxy rejects redirects, unknown query parameters, traversal and
non-canonical paths. It applies bounded upstream timeouts and response sizes,
and returns every authenticated playlist and part with `Cache-Control:
no-store, private`. Port 8888 remains unbound on the host. This avoids the
public UDP, ICE and TURN requirements of WHEP, at the cost of somewhat higher
latency and backend bandwidth. A token bucket limits unauthenticated work by
client address; authenticated upstream reads are bounded globally and per
session.

Production deployment reads `VIEWER_WORKER_PUBLISHING_ENABLED` and
`VIEWER_HLS_PROXY_ENABLED` from protected GitHub environment variables. Both
default to `false`, and deployment rejects values other than the exact strings
`true` and `false`. When HLS is enabled, the protected
`MEDIA_HLS_CDN_SECRET` secret is mandatory and must be a 43-128 character
base64url value. Enabling worker publication also requires `MEDIA_PUBLISH_IP`
to contain the VM's non-loopback IPv4 address (`10.0.0.126` in the current
topology). Deployment checks both the value and the actual Docker port binding;
it cannot silently fall back to loopback and still report success. Keep the
feature gates false for general use until a real session proves RTSP ingest,
authenticated playback, lease expiry, cleanup, and simultaneous-user stream
isolation. A supervised acceptance may enable both gates temporarily; if it
does not finish successfully, disable them again. After acceptance, set both protected variables to `true`; later
backend deployments will preserve that commissioned state. WHEP can remain a
separate optional transport.

CI and the production Compose definition pin MediaMTX 1.18.2 by its
multi-platform SHA-256 manifest digest, not only by its mutable version tag.
Before replacement, rollback records the running container's immutable image
ID. A later registry-tag change therefore cannot silently alter either the
tested binary or the image restored after a failed deployment.

## Local commissioning result

The complete release helper was exercised against a disposable N=10 server
with Gazebo Classic 11.15.1. Its original image-built probe reported D3D12 on
an NVIDIA GeForce RTX 3080, 46.124 camera FPS, 49.785 PostRender FPS at the
50 FPS cap, physics RTF 2.975, and a 990x588 viewport inside a verified
1280x720 fullscreen window. Local RTSP decoded H.264 1280x720 at 30 FPS; the
captured scene and ROS roster showed `tb3_0` through `tb3_9`. Protocol-2 input
moved the real X pointer and completed button/key down-up pairs. A final
SIGKILL of the helper PGID removed Xvfb, FFmpeg, the outer bwrap monitor, and
the inner `--new-session` bwrap/gzclient tree through `--die-with-parent`; no
display lock, runtime directory, container, or network survived cleanup.
Production browser authentication, lease expiry, and simultaneous public-user
isolation remain separate commissioning gates.

## Tests

The test uses real host Xvfb/X11/XTest with fake Docker metadata, gzclient,
window inspection, FFmpeg, and a local RTSP test server; it does not require a
GPU. It runs two publishers concurrently and checks separate cookie-protected
abstract Unix displays, no X11 TCP listeners, real pointer injection, exact
container/private-network endpoint validation, packaged sandbox mounts,
render-report PID/GPU/cap/viewport validation, sustained RTP, canonical paths,
signal cleanup, stale X-lock recovery after SIGKILL, exclusive lease ownership,
worker-label binding, and token isolation.

```bash
deploy/gpu-worker/test-viewer-publisher.sh
```

For a real host probe (which does not start a session or publish media), the
versioned plugin/assets and host dependencies must already be beside the
helper:

```bash
deploy/gpu-worker/robotswarm-viewer-publisher probe \
  --protocol-version 2 \
  --publish-base-url 'rtsp://127.0.0.1:8554'
```

The probe reports interactive H.264 `Scene` capability only when Docker, host
Gazebo 11, bubblewrap, Xvfb/X11/XTest, the packaged GUI assets/plugin, and a
supported FFmpeg encoder are present. It does not replace the real
GPU/RTSP/browser benchmark.
