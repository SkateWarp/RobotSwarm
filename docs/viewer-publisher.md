# Scene viewer publisher

`deploy/gpu-worker/robotswarm-viewer-publisher` implements protocol 1 of the
SwarmWorker external viewer contract. It exposes `Scene` only. The TurtleBot3
Burger image does not yet contain a supported robot-camera sensor, so the
helper deliberately rejects `RobotCamera`.

The GPU-worker workflow syntax-checks and tests this file, packages it with
mode `0755`, and gives each versioned release an absolute
`Worker__Viewer__PublisherExecutable` path. The owner-only identity file keeps
viewer publishing disabled by default.

For each viewer lease, the helper:

1. takes an advisory lock on an unused private X display and creates a unique
   MIT-MAGIC-COOKIE Xauthority file;
2. starts a private Xvfb server;
3. verifies the worker-owned session container and its immutable image ID;
4. runs `gzclient` in a disposable sidecar made from that exact image and joins
   the session container's network namespace, where the Gazebo master is
   available at `127.0.0.1:11345`;
5. waits until the Gazebo window exists, then captures the private display with
   FFmpeg and publishes the canonical session path to MediaMTX over RTSP; and
6. prints `READY` only after the display, gzclient, and publisher have remained
   alive through the startup settling period.

The short-lived, publish-only lease token is sent as one bounded line on the
helper's standard input. The worker flushes and closes that pipe immediately;
the probe receives closed, empty input. The token is never placed in the
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
Xvfb, gzclient, and FFmpeg. If the helper itself is killed, the worker sends
`SIGTERM` and then `SIGKILL` to that exact group and verifies that it is gone
before starting a replacement. Recovery fails closed if the group cannot be
reaped. A mode-0600 per-lease lock prevents concurrent ownership, while a
same-UID mode-0700 runtime directory can be reclaimed after a hard kill;
symlinks, unexpected owners, and permissive stale directories are rejected.

## Host requirements

Install these commands on the WSL GPU worker before running a real smoke test:

- Docker with access to the pinned RobotSwarm ROS image;
- `Xvfb` from `xvfb`, plus `xwininfo` from `x11-utils`;
- FFmpeg with `h264_nvenc` or `libx264`; and
- the NVIDIA Container Toolkit when the gzclient sidecar uses `--gpus`.

FFmpeg automatically prefers `h264_nvenc` and performs a one-frame encoder
initialization test with the selected runtime preset, tuning, rate-control,
pixel format, frame size, and frame rate during the helper probe. Set
`ROBOTSWARM_VIEWER_ENCODER=h264_nvenc` to fail closed instead of accepting the
CPU fallback, or `libx264` to select the tested software encoder explicitly.
The deployment persists this identity setting, defaulting to `auto`. The
default scene size is 1280x720 at 30 FPS and 4 Mbit/s.

The default display transport is a Unix X socket. WSLg mounts
`/tmp/.X11-unix` read-only, however, so a private Xvfb server cannot create a
socket there. Set `ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=tcp` on that host. Xvfb
then disables its Unix listener, retains MIT-MAGIC-COOKIE authentication, and
listens on TCP port `6000 + display`. Host capture uses
`127.0.0.1:<display>`. The helper verifies the session's single worker-owned
internal Docker network, inspects its IPAM gateway, and gives gzclient
`<gateway>:<display>` without mounting the X socket. Network ID, name, owner,
subnet, and gateway mismatches fail closed.

Xvfb can listen beyond the Docker bridge. Restrict the configured display-port
range with the host firewall and do not expose it to the LAN or Internet. The
Xauthority cookie remains mandatory, but is not a replacement for network
filtering. TCP mode works with `PrivateTmp=true`; Unix mode still requires a
reviewed `PrivateTmp=false` worker unit so Docker can see the socket.

Optional settings are local host configuration, never viewer-command values:

```bash
ROBOTSWARM_VIEWER_DISPLAY_BACKEND=xvfb
ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=tcp
ROBOTSWARM_VIEWER_DISPLAY_SERVER=/usr/bin/Xvfb
ROBOTSWARM_VIEWER_ENCODER=auto
ROBOTSWARM_VIEWER_GZCLIENT_GPU_REQUEST=device=0
ROBOTSWARM_VIEWER_WIDTH=1280
ROBOTSWARM_VIEWER_HEIGHT=720
ROBOTSWARM_VIEWER_FRAME_RATE=30
ROBOTSWARM_VIEWER_BIT_RATE=4M
```

The helper's default startup budget is 12 seconds. Keep
`Worker__Viewer__StartupTimeoutSeconds` at least a few seconds higher, and raise
both values together if the pinned image needs longer to create its Gazebo
window. `READY` is never emitted after the helper's own budget expires.

The deployment already installs the helper and points
`Worker__Viewer__PublisherExecutable` at its absolute, versioned path. Setting
`Worker__Viewer__Enabled=true` in the identity file makes deployment require a
credential-free RTSP base URL and a successfully initialized H.264 encoder.
The deployment passes the configured `unix` or `tcp` transport to the same
probe and preserves it in the release environment. The helper prefers NVENC
and can fall back to `libx264`; keep the
backend's worker-publishing gate off until RTSP ingest, external WHEP/ICE/TURN,
lease expiry, and simultaneous-session isolation all pass.

## Local commissioning result

Two live session pipelines ran concurrently with separate X displays,
gzclient sidecars, locks, runtime directories, and RTSP paths. Each stream
decoded 90 H.264 frames over three seconds at 960x540 and 30 FPS, while the two
Gazebo servers held real-time factors of 2.995 and 2.996. A marker placed in
the second display appeared only in the second stream, and stopping its lease
left the first stream running. Process inspection found no publish token in a
helper, proxy, or FFmpeg argument/environment.

That host's FFmpeg 4.2.7 build could not initialize NVENC with the required
runtime options, so `auto` correctly selected `libx264`; Gazebo rendering still
used the NVIDIA GPU. Keep the production viewer gate disabled until the public
WHEP hostname and ICE/TURN path are commissioned.

## Tests

The test uses fake Docker, X server, window inspector, and FFmpeg commands. It
does not require a GPU or media server. It runs two publishers concurrently and
checks that they use different displays, the exact session image and network,
the inspected network gateway, authenticated TCP X11 routing, NVENC preference,
canonical paths, signal cleanup, hard-kill runtime recovery, exclusive lease
ownership, and token isolation.

```bash
deploy/gpu-worker/test-viewer-publisher.sh
```

For a real host probe (which does not start a session or publish media), only
the configured RTSP base URL is required:

```bash
deploy/gpu-worker/robotswarm-viewer-publisher probe \
  --protocol-version 1 \
  --publish-base-url 'rtsp://127.0.0.1:8554'
```

The probe reports H.264 `Scene` capability only when Docker, the selected X
server, `xwininfo`, and a supported FFmpeg encoder are present. It does not
replace the real multi-session GPU/RTSP/WHEP benchmark.
