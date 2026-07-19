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
and can fall back to `libx264`; keep the backend's worker-publishing gate off
until RTSP ingest, one browser delivery path, lease expiry, and
simultaneous-session isolation all pass.

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
feature gates false until a real session proves RTSP ingest,
authenticated playback, lease expiry, cleanup, and simultaneous-user stream
isolation. After acceptance, set both protected variables to `true`; later
backend deployments will preserve that commissioned state. WHEP can remain a
separate optional transport.

CI and the production Compose definition pin MediaMTX 1.18.2 by its
multi-platform SHA-256 manifest digest, not only by its mutable version tag.
Before replacement, rollback records the running container's immutable image
ID. A later registry-tag change therefore cannot silently alter either the
tested binary or the image restored after a failed deployment.

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
used the NVIDIA GPU. Keep the production viewer gates disabled until two
independent public browser sessions pass through the authenticated backend HLS
proxy with cross-user denial, lease expiry, and independent cleanup. Public
WHEP/ICE/TURN is optional and requires a separate acceptance test only if that
transport is enabled later.

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
replace the real multi-session GPU/RTSP/browser benchmark.
