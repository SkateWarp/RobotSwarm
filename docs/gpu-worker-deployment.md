# GPU worker deployment

The GPU worker has its own deployment path. It does not deploy the React
frontend or the backend:

- `rs.zerav.la` remains a Cloudflare-hosted frontend deployed by Cloudflare's
  Git integration.
- `robot.zerav.la` remains the backend/control-plane host and uses the backend
  workflow.
- The GPU workflow runs only on the dedicated WSL machine and updates the
  outbound `SwarmWorker` service plus its local ROS/Gazebo image.

This separation matters because a frontend commit must not restart an active
simulation host, and a worker/ROS commit must not modify the backend VM.

## One-time host setup

Use a dedicated WSL user for both the GitHub Actions runner and the worker
service. The workflow intentionally deploys a **user** systemd service, so the
runner and service must have the same home directory.

The host needs:

- Windows 11, WSL2, and
  [systemd enabled](https://learn.microsoft.com/en-us/windows/wsl/systemd)
  in the selected distribution.
- Docker Desktop's WSL engine and integration enabled for that distribution.
- A Windows NVIDIA driver with
  [WSL GPU support](https://docs.nvidia.com/cuda/wsl-user-guide/index.html).
- A
  [self-hosted GitHub Actions runner](https://docs.github.com/en/actions/how-tos/manage-runners/self-hosted-runners/use-in-a-workflow)
  registered with `--no-default-labels` and only these custom labels:
  `wsl`, `gpu`, and `gpu-worker-prod`.
- .NET is downloaded by the workflow. The deployed worker is self-contained
  and does not depend on the runner's .NET installation.

Do not give the GPU runner the generic `self-hosted` label. The backend VM has
its own `backend-prod` label; keeping the label sets disjoint prevents GitHub
from scheduling a backend/database deployment on the Windows GPU machine.

Enable systemd in `/etc/wsl.conf`, restart WSL, and keep the user manager
available without an interactive login:

```ini
[boot]
systemd=true
```

```bash
sudo loginctl enable-linger "$USER"
systemctl --user show-environment
docker info
docker run --rm --gpus device=0 \
  nvidia/cuda:12.6.3-base-ubuntu22.04 nvidia-smi
```

The GitHub runner must be installed as a persistent service inside this WSL
distribution. Do not point the runner's work directory at a manually edited
checkout. A system runner service may not inherit `XDG_RUNTIME_DIR` or
`DBUS_SESSION_BUS_ADDRESS`; the workflow and deployment script derive both
from `/run/user/<uid>` before calling `systemctl --user`.

Linger keeps the user manager available inside a running distribution, but it
does not start a stopped WSL VM or Docker Desktop after Windows reboots.
Configure Windows startup to start Docker Desktop and this WSL distribution
before expecting the runner or worker to be online.

## Provision worker identity once

Enroll the worker through the backend, then create the identity file locally.
It is not stored as a GitHub secret or committed to the repository:

```bash
install -d -m 0700 "$HOME/.config/robotswarm"
install -m 0600 \
  deploy/gpu-worker/gpu-worker-identity.env.example \
  "$HOME/.config/robotswarm/gpu-worker-identity.env"
```

Edit the copied file and replace every placeholder. Its capacity values must
match the enrolled worker values used by backend scheduling.

The deployment script refuses symlinks, the all-zero worker ID, template
secrets, files owned by another user, and files readable by group or other
users. Do not print or upload this file during diagnostics.

The runner and worker currently share one Unix user, so approved workflow code
can still read this file and use the Docker socket. Treat the production runner
as privileged infrastructure: protect `main`, disallow unreviewed workflow
changes, never run pull-request code on this runner, and require approval on a
GitHub environment named `gpu-worker-production`. The workflow pins its
third-party actions to full commit SHAs.

Make environment approval and deployment-branch restrictions mandatory, not
optional. The optional GitHub environment variable
`GPU_WORKER_GPU_REQUEST` can select a GPU; it defaults to `device=0`.

## What a deployment does

`.github/workflows/gpu_worker_workflow.yml` is maintenance-only and must be
started manually. The operator supplies the full main-branch commit SHA and
confirms that any corresponding backend rollout is already compatible and
healthy. The workflow itself acquires a worker-authenticated drain lease tied
to that exact SHA; no checkbox is accepted as evidence that scheduling has
stopped.

The workflow:

1. Uses the GitHub Actions API to require a successful `Check project` push
   run for that exact SHA on `main`, requires it to still be the current
   `main` head, and checks it out as the worker/ROS source.
2. Requests a two-hour drain lease from the backend and waits up to fifteen
   minutes for both the control-plane session count and the worker's
   post-request Docker report to reach zero.
3. Separately checks out the deployment script, unit, and rollback tests from
   the protected workflow SHA. A source selection cannot replace today's
   deployment safety harness with an older copy.
4. Confirms the runner is WSL, Docker is available, the systemd user manager
   is reachable, the local identity file exists, and no managed simulation
   container is running.
5. Runs the .NET worker tests, viewer-helper tests, deployment rollback tests,
   and ROS algorithm tests again on the GPU host.
6. Builds the ROS image from `swarm_ws/Dockerfile`, labels it with the full Git
   revision, and records Docker's exact `sha256:...` image ID.
7. Smoke-tests the ROS launch file and NVIDIA access inside that exact image.
8. Publishes a self-contained Linux worker, packages the tested viewer helper
   with mode `0755`, and stages both in a versioned release under
   `~/.local/share/robotswarm-gpu-worker/releases/`.
9. Revalidates the lease, target SHA, remaining lifetime and both zero-session
   reports immediately before changing the active release.
10. Atomically switches the `current` symlink, which selects both the worker
   executable and its owner-only release environment, starts
   `robotswarm-gpu-worker.service`, and requires the current service PID to log
   the post-registration, first-heartbeat readiness marker with the expected
   image version.
11. Releases the lease only after readiness succeeds, allowing the backend to
    schedule new sessions again.

Before switching releases, the deployment script checks again for running
managed containers, stops the old worker, and checks a third time. A running
session blocks deployment. The backend keeps the worker in `Draining` for the
entire interval, which closes the previous race in which a new session could
be assigned between a local check and stopping the old process.

When the selected SHA changes backend/worker contracts, deploy and verify the
backend for the same SHA first. The drain route must therefore exist before a
worker workflow that depends on it is dispatched. A broader protocol-version
compatibility handshake remains future work.

### Activating full-fleet transport evidence

The transport-result contract uses a deliberate two-stage rollout. Keep the
protected production variable
`REQUIRE_COLLABORATIVE_TRANSPORT_EVIDENCE=false` while the new backend is being
deployed in front of the previous ROS image. In this compatibility stage, a
completed transport is still judged by the earlier progress contract, so the
backend-first deployment cannot turn a valid in-flight result into a failure.

The new worker advertises
`taskOutcomes.collaborativeTransportEvidenceVersion=1`. Its ROS result includes
the finder, the exact notified companion set, and the exact useful-contributor
IDs. After the GPU workflow has drained the old worker, activated the same
current `main` SHA, and reported readiness, set the protected variable to
`true` and rerun the backend deployment for that same SHA. The backend then
refuses to start a transport on a worker without evidence version 1 and accepts
completion only when all reported identities match the session's stored robot
roster. Leave the variable true after commissioning; a worker rollback that
lacks the capability must block new transport tasks instead of silently
weakening acceptance.

Before changing the variable, verify that no session remains active. The GPU
drain lease and the shared `production-robotswarm` workflow lock provide this
maintenance boundary. Record the worker registration capability, deployment
SHA, and one accepted full-fleet result in the commissioning evidence.

The release environment records the absolute Docker executable found by the
runner and the absolute path to that release's viewer helper. This avoids
relying on an interactive WSL shell's `PATH` when the worker starts under
systemd and keeps helper rollback coupled to worker rollback. Viewer enablement
remains in the stable, owner-only identity file rather than release output.
When enabled, deployment requires the configured RTSP endpoint and a passing
H.264 helper probe before it changes the active release. Unix display mode
changes only that installed unit to `PrivateTmp=false`; authenticated TCP mode
and disabled deployments retain the checked-in `PrivateTmp=true` unit
unchanged. The probe and active service receive the same display transport,
encoder policy, and GPU request.

If activation or readiness verification fails, the script restores the prior
release symlink, release environment, unit file, enabled state, and running
state. It reports a critical error if any rollback step fails. It never
removes active ROS session containers.

Locally built Docker images normally have no repository digest. The worker is
therefore configured with Docker's exact content-addressed image ID:
`sha256:<64 hex characters>`, with
`Worker__AllowMutableSessionImage=false`. No mutable tag is passed to a
session.

Do not run broad image pruning on this host. An image ID referenced by a
deployed release must remain available so a later session can be provisioned.
Safe, bounded garbage collection for failed builds, old releases, and their
unique image tags is not automated yet.

## Operations

Check the active service and release without reading the identity file:

```bash
systemctl --user status robotswarm-gpu-worker.service
journalctl --user -u robotswarm-gpu-worker.service -n 100 --no-pager
readlink "$HOME/.local/share/robotswarm-gpu-worker/current"
cat "$HOME/.local/share/robotswarm-gpu-worker/current/DEPLOYMENT"
```

The generated release environment is stored inside each release. The active
one is available at:

```text
~/.local/share/robotswarm-gpu-worker/current/gpu-worker-release.env
```

It has mode `0600`, and the same atomic symlink switch selects the matching
binary and environment. The stable identity file is never rewritten by CI.

The maintenance workflow accepts only the current, CI-passed `main` head. Its
automatic rollback can restore the release that was active immediately before
a failed activation. There is no supported command for manually activating an
arbitrary older release. Avoid changing only the `current` symlink: the binary,
environment, locally retained image, and systemd state must remain consistent.

## Viewer commissioning boundary

The workflow proves that Docker can expose the NVIDIA GPU, the ROS image builds,
and the packaged viewer helper can initialize its configured X and H.264
dependencies. Separate host-side commissioning has also run two live publisher
pipelines concurrently with different private displays, `gzclient` sidecars,
stream paths, runtime directories, and cleanup. That is useful evidence, but it
does not by itself prove the final public browser route.

Every versioned GPU-worker release includes the Scene-only protocol-1 helper and
records its absolute executable path. Shipping it does not enable media by
default. Keep `Worker__Viewer__Enabled=false`, backend
`Viewer__WorkerPublishingEnabled=false`, and backend
`Viewer__HlsProxyEnabled=false` until all of these pass on the release revision:

- a private display and visible Gazebo client for each simultaneous session;
- the helper probe against the pinned image, selected encoder, and authenticated
  MediaMTX RTSP ingest path;
- two public, independently authenticated browser sessions through the backend
  low-latency HLS proxy, including cross-user denial, lease expiry, and
  independent stop/cleanup; and
- measured FPS, physics real-time factor, and simultaneous-session CPU/GPU
  cost on the production worker.

The primary browser route is authenticated low-latency HLS through
`robot.zerav.la`; it does not require a public WHEP hostname, ICE port, or TURN.
WHEP remains optional. If it is enabled later, its public connectivity and TURN
fallback require a separate acceptance test.

The backend and MediaMTX share one protected `MEDIA_HLS_CDN_SECRET` only on
their private Compose hop. It is not a viewer lease or a publish token. The
backend deployment refuses to enable HLS if this 43-128 character base64url
secret is absent or malformed.

The helper prefers `h264_nvenc` but accepts its tested `libx264` fallback when
the host FFmpeg cannot initialize NVENC. Record the encoder selected by the
probe and include its CPU cost in the simultaneous-session benchmark. Set
`ROBOTSWARM_VIEWER_ENCODER` in the identity file to `auto` (the default),
`h264_nvenc`, or `libx264`; deployment uses the same policy for its probe and
the active worker release.

WSLg hosts should set `ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=tcp` in the identity
file. Deployment validates and persists `unix` (the general default) or `tcp`;
TCP keeps the service's private `/tmp`, while Unix mode exposes the X socket as
documented in the publisher guide.

The stock TurtleBot3 Burger model in this repository has no camera sensor, so
the first honest helper capability is `Scene` only. Do not use the shared WSLg
root display as a shortcut: it cannot isolate simultaneous users or reliably
select the correct Gazebo window.

ROS 1 Noetic
[reached end of life on May 31, 2025](https://www.ros.org/blog/noetic-eol/).
Keep session containers isolated as designed, track the base image and package
inputs used by each build, and plan a ROS 2 migration rather than treating new
Noetic images as indefinitely supported.
