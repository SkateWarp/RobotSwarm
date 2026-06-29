# Changelog

Detailed history of the RobotSwarm repository, reconstructed from git commits.

> **Note on history depth:** the local clone is *shallow/grafted* at commit
> `7d6af26` (2025-05-13). Everything in that first commit appears as one
> "initial import" of the already-scaffolded project; commits before it are
> not present in this clone. Authors: **Félix Alejandro Guzmán** and
> **Anyelo (Alvarez)** are the human contributors; later commits are
> co-authored by Claude.

---

## Initial import — 2025-05-13 (`7d6af26`)
Grafted baseline containing the full three-tier project already scaffolded:

- **SwarmBackend** (.NET 8 Minimal API): entities (`Robot`, `Sensor`,
  `SensorReading`, `TaskLog`, `TaskTemplate`, `Account`, `RefreshToken`,
  `RobotGroup`, `Role`), EF Core `DataContext` + migrations back to
  `20230414192503_Initial`, service/interface pairs, Minimal-API route groups
  (`RobotRoute`, `SensorReadingRoute`, `TaskLogRoute`, `AccountRoute`,
  `WebsocketRoute`, …), `RobotHub` SignalR hub, JWT auth, Dockerfile,
  `docker-compose.local.yml`.
- **SwarmFrontend** (React 17 / Fuse template / MUI 5 / craco): full Fuse admin
  template, GeeTS app modules, multi-brand assets, ESLint/craco config.
- **CI**: `frontend_workflow.yml` and `backend_workflow.yml` (self-hosted).

---

## ROS bridge command handling — 2025-05-13 → 2025-06-05
Focused on the Python SignalR↔ROS bridge and command plumbing.

- `f81456a` Allow non-float sensor values in the sensor-data send path (`signalr_handler.py`).
- `12b750d` `dummy_turtlebot3.py`: string→int fixes.
- `8923a93` Refactor `TaskLogService` to take the hub context; trim unused imports in `RobotHub`.
- `e472d1e` Log received commands via `rospy` instead of the logger.
- `1fc0cd2` Log command content and notify the targeted robots.
- `140b887` / `4514f1c` Accept **multiple** command arguments (not just one) and extract the command correctly.
- `29891f3` Fix the key used to read robot IDs in command handling.
- `216511f` Add robot status to the LeafSorting form (`LeafSortingConfigDialog.js`, large rework).
- `51dce1e` `RobotService`: broadcast "robots available" on update.

---

## ROS rate-limiting & SignalR reliability — 2025-10-14 → 2025-10-17
- `9049ccc` **chore(workflows):** add Docker/system log-cleanup step to the
  backend workflow; add log size/rotation to `docker-compose.prod.yml`; add
  `SwarmBackend/mise.toml` pinning .NET 8.
- `ee5bf24` **feat(ros-handler):** type-specific rate limiting (separate
  intervals for status / sensor / task / cancel) replacing a single interval;
  `dummy_turtlebot3.py` emits sensor data as `data: {JSON}`.
- `abf11d2` Fix robot topic wiring in `multi_dummy_turtlebot3.launch` + dummy bot.
- `b28465` / `4f8fbe9` Follow-up dummy-bot fixes.
- `2407322` **feat(signalr):** `RobotWidget` waits for the SignalR connection
  before registering handlers; `signalRConnectionService` stores connection
  promises for promise-based readiness checks and proper cleanup.
- `26504d6` Convert sensor value to string in the reading request.

---

## Task Logs feature + dashboard/detail UI — 2025-10-24 → 2025-10-31
A large frontend + backend feature wave around tasks and robot detail views.

- `a3461ab` Sort sensor readings by `notes` (backend + `RobotWidget`); use
  `reading.id` as React key.
- `a6ebf34` Remove debug logs across many components.
- `4ed6512` "enviar tarea" — task-sending UI in `RealtimeConfigList` (+222 lines).
- `f309043` **feat(geets):** Task Logs feature — new `TaskLog` module
  (`TaskLogApp/Config/Dialog/List` + Redux slice), nav item with admin auth, route.
- `aacfd8d` Drop `api/` prefix from the TaskLog route.
- `523e86d` **feat(tasklog):** cancel tasks by robot **and account**
  (`CancelTasksByRobot` in service/interface/hub) + frontend running-task tracking.
- `4d69a60` Require account ID to cancel tasks (authorization on
  `IRealtimeService` / `TaskLogRoute` / frontend).
- `8a86c73` **refactor(dashboard):** CSS Grid layout; restructured MUI
  `RobotWidget` with sensor grouping (+201/−86).
- `2519acf` **feat(api):** `GetByRobot` endpoint for task logs filtered by robot.
- `9960c5e` **feat:** Robot Detail page — new `RobotDetail` module
  (`RobotDetailApp`, `RobotSensorData`, `RobotTaskLogs`, `SensorChart`), click-through from dashboard.
- `33b6899` Route tweak ("ir a dashboard").
- `7fd5538` **feat:** add `CommandPanel` + `CommandStatus` to Realtime; move
  "Create" buttons into headers; introduce `RealtimeConfigListImproved`.
- `c5344f1` Rename a config-app header to "TAREAS".

---

## Account role-based access control — 2025-11-04
- `aba74da` **feat(account):** RBAC on account retrieval — `GetAll` filters by
  role/ID, new authorized `GetById`, role added to claims, routes require auth.
- `790646c` Allow `role == null` internal `GetAll` for seeding/admin; seeding passes null.
- `4696f28` / `3d112cf` / `e915d60` / `6ee63b4` Frontend `accountsSlice`
  fixes: drop account params, update endpoint, add auth headers (Bearer),
  remove `.data` wrapper and `/api` prefix.
- `e188b41` **feat(accounts):** adapt Accounts app for the GTS project — only
  Admin/User roles, route-based filtering, sidebar relabeled "Todos".

---

## CLAUDE.md + sensor data flow + Docker fix — 2026-01-28
First Claude-assisted commits.

- `4a980de` Add initial `CLAUDE.md`.
- `44c7bb3` **refactor(realtime):** place Send Command beside the VNC preview
  (VNC 2/3 width, panel 1/3); make VNC responsive.
- `e9aad9a` **feat(sensors):** end-to-end sensor data fix —
  batch methods `HandleSensorReadingsBatch` / `HandleSensorReadingFromClient`
  (12 calls → 1), frontend routes `sensor_data` to the client handler with a
  `sensorType` dropdown, ROS publisher gets configurable `~sensor_type` param.
- `23f3015` **fix(docker):** add `dotnet restore` in the final stage so EF
  migrations have their NuGet packages.

---

## Swarm Control Panel + build/CORS fixes — 2026-03-10
The largest recent wave: a dedicated swarm-control surface plus its merge into Realtime.

- `90bf412` "Coffee and changes." — adds the reference docs
  (`ARCHITECTURE.md`, `BACKEND_INTEGRATION.md`, `FRONTEND_API.md`,
  `IMPLEMENTATION_STATUS.md`, `IMPLEMENTATION_SUMMARY.md`, `QUICKSTART.md`),
  root `docker-compose.yml`, backend `RosBridgeService.cs` (+315) and
  `RobotHub` additions (+203), and a TS swarm stack on the frontend
  (`SwarmControlPanel.tsx`, `useSwarmControl.ts`, `SwarmService.ts`,
  `SwarmControlPanel.js` +970, `SwarmControl` app/config).
- `d8fa711` **Fixed build errors:** convert the TS swarm files to JS
  (`.tsx/.ts → .js`).
- `0181d2c` Fix CORS in `Program.cs`.
- `7cd21f9` **migration fix:** `20260310000000_AddRobotNamespace` + snapshot.
- `f8e17ba` **Mashup swarm controller and realtime:** merge `SwarmControlPanel`
  into the Realtime UI; rework `CommandPanel` / `CommandStatus` / `RealtimeConfigListImproved`.
- `e5c0198` / `e7d6fa0` Small swarm-panel and connection fixes.
- `aa10f8c` Fix realtime panel scroll / tab overflow (flex + bounded maxHeight).
- `be5aa59` Let Fuse `innerScroll` own overflow (remove fighting `h-full`/maxHeight).
- `e1d82d1` Disable all swarm action buttons (deploy, delete, task start/stop,
  e-stop) unless **both** server and ROS bridge are connected.

---

## Documentation (this branch `claude/init-mgnmcn`) — 2026-06-29
- `874b4a8` Improve `CLAUDE.md`: document `firmware/` and `ros_wss/`, ROS Noetic
  + Gazebo, `start.sh`, root `docker-compose.yml`, `mise.toml`, backend workflow,
  reference docs.
- `f136908` Drop ESP32 firmware and HeRo platform references from `CLAUDE.md`.

---

### Themes at a glance
| Area | Key commits |
|------|-------------|
| ROS bridge / command handling | `f81456a`–`51dce1e`, `ee5bf24`, `abf11d2` |
| Sensor data flow | `26504d6`, `a3461ab`, `e9aad9a` |
| Task logs & cancellation | `f309043`, `523e86d`, `4d69a60`, `2519acf` |
| Robot dashboard / detail UI | `8a86c73`, `9960c5e`, `7fd5538` |
| Account RBAC | `aba74da`, `790646c`, `e188b41` |
| Swarm control panel | `90bf412`, `f8e17ba`, `e1d82d1` |
| Infra (Docker/CORS/CI/migrations) | `9049ccc`, `23f3015`, `0181d2c`, `7cd21f9` |
| Docs | `4a980de`, `90bf412`, `874b4a8`, `f136908` |
