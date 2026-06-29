# Evolution of the RobotSwarm Platform
### A Software-Archaeological Report on the Version-Control History of a Three-Tier Robotic Swarm Management System

**Repository:** `SkateWarp/RobotSwarm`
**Period under study:** 13 May 2025 – 29 June 2026
**Document version:** 1.0
**Compiled from:** the git commit graph of branch `claude/init-mgnmcn` and its ancestors

---

## Abstract

This report presents a detailed, commit-by-commit reconstruction of the
development history of *RobotSwarm*, a robotic swarm management platform composed
of a React front end (`SwarmFrontend`), a .NET 8 minimal-API back end
(`SwarmBackend`), and a ROS Noetic workspace (`swarm_ws`) bridged to the back end
over SignalR. Working exclusively from the version-control record, we identify
eight discrete development epochs, analyse the engineering motivation behind each,
and examine the most architecturally consequential changes at the level of
individual source diffs. We further provide a quantitative analysis of commit
cadence, authorship, and file churn, and we catalogue the technical debt that the
history reveals — including duplicated TypeScript/JavaScript implementations, a
hosted service that was authored but never registered in the dependency-injection
container, and the accidental committing of `node_modules` shims. The report is
intended both as a narrative changelog and as an onboarding artefact for engineers
new to the codebase.

---

## Table of Contents

1. Introduction
   1.1 Purpose and scope
   1.2 Methodology
   1.3 Limitations of the source record
2. The Architectural Baseline
   2.1 The three-tier topology
   2.2 The back-end composition root
   2.3 The SignalR contract
3. Development Epochs
   3.1 Epoch I — ROS-bridge command-handling stabilisation (May–Jun 2025)
   3.2 Epoch II — Real-time reliability and type-aware rate limiting (Oct 2025)
   3.3 Epoch III — The task-lifecycle subsystem (Oct 2025)
   3.4 Epoch IV — Dashboard and robot-detail experience (Oct 2025)
   3.5 Epoch V — Account role-based access control (Nov 2025)
   3.6 Epoch VI — The sensor-data pipeline overhaul (Jan 2026)
   3.7 Epoch VII — The swarm-control plane (Mar 2026)
   3.8 Epoch VIII — Documentation and tooling (Jan / Mar / Jun)
4. Quantitative Analysis
   4.1 Authorship
   4.2 Commit cadence and temporal clustering
   4.3 File churn and architectural hotspots
5. Technical Debt and Observations
6. Conclusion
- Appendix A — Complete commit ledger
- Appendix B — Methodological notes

---

## 1. Introduction

### 1.1 Purpose and scope

The objective of this document is to transform an opaque sequence of 57 git
commits into an intelligible account of *how* and *why* the RobotSwarm platform
arrived at its present shape. Where a conventional changelog enumerates *what*
changed, this report additionally reconstructs the engineering intent behind each
wave of work, situates individual commits within larger thematic campaigns, and
draws out the cross-cutting consequences — both beneficial and detrimental — that
only become visible when the history is read as a whole.

The scope is the entire reachable history of the repository as cloned, spanning
from the grafted baseline commit `7d6af26` (13 May 2025) to the documentation
commit `f136908` (29 June 2026), together with the two commits added on the
working branch during the preparation of this report.

### 1.2 Methodology

The analysis proceeded in three passes. The first pass enumerated the commit graph
(`git log --graph --all`) to establish chronology, branch topology, and merge
structure. The second pass extracted per-commit metadata and filtered diff
statistics (`git show --numstat`), deliberately excluding high-volume noise — the
generated `package-lock.json`, binary image and font assets, EF Core
`*.Designer.cs` migration scaffolding, and the 17,693-line `prompts.txt` — so that
genuine engineering signal could be isolated. The third pass performed close
reading of the unified diffs of the commits judged most architecturally
significant, in order to ground the narrative in actual code rather than commit
messages alone.

### 1.3 Limitations of the source record

Three limitations must be stated plainly.

**Graft horizon.** The clone is *shallow and grafted* at commit `7d6af26`. That
commit is marked `(grafted)` and presents the project as an already-substantial,
fully-scaffolded codebase — the entire Fuse React admin template, a complete EF
Core schema with migrations dating to April 2023, and a working SignalR hub.
Consequently, the genesis of the project and an unknown quantity of prior history
lie *below the graft horizon* and are unrecoverable from this clone. All statements
about "the baseline" therefore describe the project *as of* 13 May 2025, not its
true origin.

**Commit-message heterogeneity.** The history mixes three commit-message registers:
terse Spanish-language messages (`"enviar tarea"`, `"quitar logs"`, `"ir a
dashboard"`), conventional-commits-style English (`feat(...)`, `fix(...)`,
`refactor(...)`), and Claude-co-authored messages with structured bodies. This
unevenness means that the *intent* of the terser commits had to be inferred from
their diffs rather than read from their subjects.

**Merge opacity.** Three merge commits join parallel lines of work between the two
principal human authors. Merges carry no independent diff and so contribute
chronology but not content.

---

## 2. The Architectural Baseline

### 2.1 The three-tier topology

As of the graft horizon, RobotSwarm already realised the topology that persists to
the present:

```
React Frontend  <--SignalR-->  .NET Backend  <--SignalR-->  ROS Bridge  <--ROS Topics-->  Robots
   (Fuse/MUI)                  (RobotHub)                   (bridge.py)
```

The back end was the system's hub in both senses: architecturally it was the star
centre through which all real-time traffic flowed, and the `RobotHub` SignalR class
was simultaneously the persistence-aware service layer *and* the real-time
transport. The baseline already contained the full entity model (`Robot`, `Sensor`,
`SensorReading`, `TaskLog`, `TaskTemplate`, `Account`, `RefreshToken`,
`RobotGroup`, `Role`), the EF Core `DataContext` with a migration lineage reaching
back to `20230414192503_Initial`, a service/interface pair per domain area, the
minimal-API route groups, JWT authentication, and a Dockerfile. The front end was a
complete instance of the Fuse admin template carrying a large inventory of demo
assets unrelated to robotics.

### 2.2 The back-end composition root

`SwarmBackend/Program.cs` is the composition root and repays close reading because
it encodes several decisions that shape everything downstream:

- **SignalR is configured with camelCase JSON** (`JsonNamingPolicy.CamelCase`,
  case-insensitive) and a 15-second keep-alive and handshake timeout. This camelCase
  policy is the silent contract that the Python bridge and the React client must both
  honour, and several later commits are best understood as corrections to that
  contract.
- **CORS is maximally permissive** — `SetIsOriginAllowed(_ => true)` together with
  `AllowCredentials()`. This is the permissive-origin-with-credentials configuration
  that the later "Fix CORS" commit (`0181d2c`) would adjust.
- **`RobotHub` is registered as a *scoped* `IRealtimeService`.** Registering a
  SignalR `Hub` subclass as an injectable scoped service is unusual; it means the
  hub serves double duty as both transport and business-logic surface, a coupling
  that the history never unwinds and that explains why so much logic accretes inside
  `RobotHub.cs`.
- **Database seeding runs at start-up** inside a service scope, creating a default
  account when the `Accounts` table is empty. A later RBAC change (§3.5) would have
  to thread `null` arguments through this seeding path to bypass its own
  authorisation filter.

### 2.3 The SignalR contract

The baseline hub exposed the client-callable methods and broadcast events that the
project's `CLAUDE.md` later tabulated: `UpdateStatus`, `HandleSensorReading`,
`SendCommand`, `HandleTaskLog`/`HandleFinishTaskLog`/`HandleCancelTaskLog` on the
inbound side; and `RobotStatusChanged/{id}`, `NewSensorReading`,
`AllSensorReadings/{id}`, `ExecuteCommand`, `RobotsAvailable`, `NewTaskLog`,
`RobotCreated` on the broadcast side. The command path is deliberately simple: a
client invokes `SendCommand(robotId, command, parameters)` and the hub re-broadcasts
an `ExecuteCommand` envelope to `Clients.All`. Much of Epoch I is the story of
making the *Python* end of this contract robust.

---

## 3. Development Epochs

The 55 post-baseline commits resolve naturally into eight epochs. The epochs are not
of uniform size — two of them (III and VII) account for the majority of net new
code — but each represents a coherent engineering campaign with a single dominant
motivation.

### 3.1 Epoch I — ROS-bridge command-handling stabilisation (May–Jun 2025)

*Commits: `f81456a`, `12b750d`, `8923a93`, `e472d1e`, `1fc0cd2`, `140b887`,
`4514f1c`, `29891f3`, `216511f`, `51dce1e` (10 commits)*

The earliest reachable work is a tight, iterative debugging campaign concentrated
almost entirely in a single file:
`swarm_ws/.../handlers/signalr_handler.py`. Read in sequence, the commit subjects
narrate a developer converging on a correct command-deserialisation routine:
improve logging (`e472d1e` switches from a generic logger to `rospy`), then enrich
the log to include command content and notify the *targeted* robots (`1fc0cd2`),
then generalise the handler to accept *multiple* arguments rather than one
(`140b887`), then correct the extraction of the command from those multiple
arguments (`4514f1c`), then fix the dictionary key used to read robot IDs
(`29891f3`). This is the signature of live integration debugging against a running
ROS/SignalR system, where each commit is a hypothesis tested against robot
behaviour.

Two commits in this epoch reach outside the bridge. `8923a93` refactors
`TaskLogService` to receive the SignalR hub context (`IHubContext<RobotHub>`),
foreshadowing the task subsystem of Epoch III by giving the service the ability to
broadcast. `51dce1e` makes `RobotService` broadcast a "robots available" event on
update, tightening the feedback loop between persistence changes and the real-time
clients. The lone front-end change, `216511f`, is a substantial rework of the
LeafSorting configuration dialog to surface robot status in the form.

The data-typing commits `f81456a` ("allow non-float sensor values") and `12b750d`
("str to int") are early skirmishes in what would become a recurring theme: the
impedance mismatch between loosely-typed ROS/JSON payloads and the strongly-typed
C# entity model. The same theme resurfaces in Epoch II (`26504d6`, "convert sensor
value to string") and is only systematically resolved in Epoch VI.

### 3.2 Epoch II — Real-time reliability and type-aware rate limiting (Oct 2025)

*Commits: `9049ccc`, `ee5bf24`, `abf11d2`, `b284657`, `4f8fbe9`, `2407322`,
`26504d6` (7 commits)*

After a four-month gap in the reachable history, work resumes with a focus on the
*quality* of the real-time channel rather than its mere function.

The centrepiece is `ee5bf24`, which replaces a single global rate-limit interval in
the ROS handler with a *per-message-type* policy:

```python
# Before:  a single interval governed every outbound message
self.rate_limit_interval = 5.0

# After:   each message class gets an interval matched to its semantics
self.rate_limit_intervals = {
    'status': 5.0,      # Status changes are infrequent
    'sensor': 1.0,      # Sensor data needs real-time processing
    'task': 5.0,        # Task messages are event-based
    'finish_task': 1.0, # Task completion should be timely
    'cancel_task': 1.0  # Task cancellation should be timely
}
```

This is a textbook separation of concerns applied to a throttling policy: the
insight is that a 5-second cadence appropriate for slowly-varying status is
actively harmful for sensor telemetry and for event-driven task transitions, which
must propagate promptly. The dispatch loop was correspondingly rewritten so that
each branch reads its own interval from the dictionary. The commit simultaneously
fixed the sensor-data wire format (`data: {JSON}`) so that the dummy robot's output
matched what the handler expected to parse — another contract-alignment fix in the
spirit of Epoch I.

The second pillar of this epoch, `2407322`, addresses a classic real-time race on
the *front end*. Prior to this change, `RobotWidget` could register SignalR event
handlers before the underlying connection had finished its asynchronous handshake,
silently dropping early events. The fix introduces **promise-based connection
readiness** into the connection singleton:

```javascript
// connectionPromises now caches the start() promise per group,
// so consumers can await readiness instead of racing the handshake.
connectionPromises[connectionKey] = connection.start()
    .then(() => { /* register close handler */ return connection; })
    .catch(err => { delete connections[connectionKey];
                    delete connectionPromises[connectionKey];
                    throw err; });
// ...
const getConnectionPromise = (groupNames) =>
    connectionPromises[groupNames || 'default']
    || Promise.reject(new Error(`No connection found for ${groupNames}`));
```

By exposing `getConnectionPromise`, the service lets widgets defer handler
registration until the connection is genuinely established, and it cleans up both
the connection and its promise on close or failure. This is the most important
reliability change in the front-end history.

The remaining commits are operational and tactical: `9049ccc` hardens the
deployment by adding Docker/system-log cleanup to the back-end CI workflow,
introducing log size and rotation limits in `docker-compose.prod.yml`, and pinning
the .NET 8 toolchain via `mise.toml`; `abf11d2`, `b284657`, and `4f8fbe9` are a
short corrective sequence on the dummy-robot launch wiring.

### 3.3 Epoch III — The task-lifecycle subsystem (Oct 2025)

*Commits: `a3461ab`, `a6ebf34`, `4ed6512`, `f309043`, `aacfd8d`, `523e86d`,
`4d69a60` and the closely-related UI commits `8a86c73`, `2519acf`, `9960c5e`,
`33b6899`, `7fd5538`, `c5344f1` (Oct 24–31)*

This is the first of the two largest epochs, and it delivers a complete vertical
slice: the ability to start, list, inspect, and *cancel* robot tasks, end to end.

The feature opens with `f309043`, which introduces the **Task Logs module** on the
front end — `TaskLogApp`, its routing config, a list view, a detail dialog, and a
dedicated Redux slice (`taskLogSlice.js`, 165 lines) — wired into navigation behind
an admin-authorisation gate. The back end gains the corresponding read paths in two
steps: `f309043`/`aacfd8d` settle the route prefix (added, then removed, for
consistency with sibling endpoints), and `2519acf` adds a `GetByRobot` query that
returns a robot's task logs with their related `TaskTemplate` and `Robot` data
eagerly included.

The epoch's most security-relevant work is the task-*cancellation* path, delivered
across `523e86d` and `4d69a60`. The service method `CancelTasksByRobot` is notable
for embedding an **ownership predicate directly in the query**:

```csharp
var runningTasks = await context.TaskLogs
    .Include(t => t.Robots).Include(t => t.TaskTemplate)
    .Where(t => t.DateFinished == null && t.DateCancelled == null)
    .Where(t => t.Robots.Any(r => r.Id == robotId && r.AccountId == accountId))
    .ToListAsync();
```

A task is cancellable only if it is both *active* (`DateFinished` and
`DateCancelled` are null) and *owned* (one of its robots has the requesting
account's `AccountId`). On success the method stamps `DateCancelled`, persists, and
transitions the robot to `RobotStatus.Idle`. The follow-up commit `4d69a60` pushes
this account-scoping all the way to the edge: the `IRealtimeService` signature, the
`TaskLogRoute`, and the front-end caller are all amended to carry the account ID, so
that a user cannot cancel another user's tasks. The use of `LanguageExt`'s
`Result<T>` for the success/error duality is consistent with the back end's
functional error-handling idiom.

Surrounding this core are the UX commits that make the subsystem usable: `8a86c73`
re-platforms the dashboard from Flexbox to CSS Grid and restructures `RobotWidget`
with grouped sensor display; `9960c5e` adds the **robot-detail page**
(`RobotDetailApp`, `RobotSensorData`, `RobotTaskLogs`, and a `SensorChart`
visualisation) reachable by clicking a widget; and `7fd5538` introduces the
`CommandPanel`/`CommandStatus` components and the `RealtimeConfigListImproved`
variant — the latter being significant because it forks the Realtime list into an
"improved" parallel implementation that subsequent epochs would continue to evolve
*instead of* the original, leaving two coexisting list components.

The minor commits `a6ebf34` ("quitar logs") and `c5344f1` (a header relabel to
"TAREAS") are housekeeping. `4ed6512` ("enviar tarea") adds 222 lines of
task-dispatch UI to the *original* `RealtimeConfigList`, illustrating the
divergence noted above.

### 3.4 Epoch IV — Dashboard and robot-detail experience

This epoch is not temporally separate from Epoch III — the commits interleave on
24–31 October — but it is conceptually distinct: where Epoch III delivered task
*function*, the commits `8a86c73`, `9960c5e`, `33b6899`, and `7fd5538` delivered the
*presentation* through which that function is exercised. They are called out
separately because they establish two durable front-end patterns: (i) navigation
from aggregate (dashboard) to detail (per-robot) views, and (ii) client-side
charting of sensor history via `SensorChart.js`. The `RobotWidget` component becomes
a churn hotspot here (see §4.3), accumulating click-through navigation, hover
affordances, client-side sorting, and a Material-UI restructuring across several
commits.

### 3.5 Epoch V — Account role-based access control (Nov 2025)

*Commits: `aba74da`, `790646c`, `4696f28`, `3d112cf`, `e915d60`, `6ee63b4`,
`e188b41` (7 commits, all 4 Nov 2025)*

This epoch retrofits authorisation onto the account subsystem in a single focused
day of work. The pivotal commit `aba74da` makes three coordinated changes:

1. **Claims enrichment.** The JWT claim set gains the user's role
   (`new(ClaimTypes.Role, account.Role.ToString())`), without which downstream
   authorisation checks would have nothing to read.
2. **Query-level filtering.** `GetAll` is re-signed to take `(int? accountId,
   Role? role)` and returns only the caller's own account unless the caller is an
   `Admin`:

   ```csharp
   if (role != Role.Admin && accountId.HasValue)
       return await _dataContext.Accounts
           .Where(x => x.Id == accountId.Value)
           .Select(x => AccountResponse.From(x)).ToListAsync();
   // Admins fall through to the unfiltered query.
   ```
3. **Route-level enforcement.** The account routes gain `.RequireAuthorization()`,
   and a new `GetById` endpoint returns `Results.Forbid()` when a non-admin requests
   an account other than their own.

The nullable `role` parameter is the seam through which internal callers bypass the
filter, and `790646c` exploits exactly this: the start-up seeding path in
`Program.cs` calls `GetAll(null, null)` so that the empty-database check is not
itself subjected to the new role filter. This is a pragmatic but subtle coupling —
the authorisation policy is now expressed partly as "a `null` role means trusted
internal caller", a convention that is easy to violate by accident.

The remaining five commits are the front-end counterpart: `accountsSlice.js` is
repeatedly adjusted to add `Authorization: Bearer` headers (`e915d60`), correct the
endpoint (`3d112cf`), drop request parameters (`4696f28`), and unwrap responses that
no longer carry a `.data` envelope (`6ee63b4`). The epoch closes with `e188b41`,
which specialises the generic Accounts app for the "GTS" project: it removes the
`Operator` role, restricts the visible roles to Admin/User, and adds route-based
filtering — an early sign that the front end is being tailored from a generic
template toward a specific deployment.

### 3.6 Epoch VI — The sensor-data pipeline overhaul (Jan 2026)

*Commits: `4a980de`, `44c7bb3`, `e9aad9a`, `23f3015` (28 Jan 2026)*

After a two-month gap, work resumes — now with Claude co-authorship — and tackles
the sensor-data path that the earlier type-coercion commits had only patched
locally. `e9aad9a` is the definitive resolution. It adds two new ingestion methods
to `RobotHub` and `SensorReadingService`:

- **`HandleSensorReadingsBatch`** for the ROS bridge, which submits *all* of a
  sensor's fields in a single hub call. The corresponding `CreateBatch` service
  method performs find-or-create on the `Sensor` row (keyed by robot and
  `SensorType`), then bulk-inserts one `SensorReading` per field via `AddRange`,
  with the field name stored in `Notes`:

  ```csharp
  var readings = request.Fields.Select(field => new SensorReading {
      SensorId = sensor.Id, Value = field.Value,
      DateCreated = DateTime.UtcNow, Notes = field.FieldName,
  }).ToList();
  context.SensorReadings.AddRange(readings);
  ```
- **`HandleSensorReadingFromClient`** for the React front end, which accepts a
  `Dictionary<string,object>` of sensor fields directly from the UI.

Both broadcast each new reading as `NewSensorReading` and then push the full
`AllSensorReadings/{robotId}` snapshot. The commit message records the headline
efficiency win: the ROS bridge now performs **one** hub call per sensor instead of
twelve. The robot publisher was simultaneously made configurable via a
`~sensor_type` ROS parameter so a single node can represent different sensor types,
and the front-end command path was rerouted to call
`HandleSensorReadingFromClient` rather than overloading the generic `SendCommand`.
The new interface methods were added to `IRealtimeService` to keep the contract
honest.

The epoch's other commits are supporting: `44c7bb3` rearranges the Realtime layout
so the command panel sits beside the VNC preview and makes the VNC viewer
responsive within its container; `4a980de` adds the first `CLAUDE.md`; and `23f3015`
fixes a real deployment defect — the Docker *final* stage lacked the NuGet packages
needed to run EF migrations, resolved by adding an explicit `dotnet restore`.

### 3.7 Epoch VII — The swarm-control plane (Mar 2026)

*Commits: `90bf412`, `d8fa711`, `0181d2c`, `7cd21f9`, `f8e17ba`, `e5c0198`,
`abf11d2`-line merges, `e7d6fa0`, `aa10f8c`, `be5aa59`, `e1d82d1` (10 Mar 2026)*

This is the largest and most ambitious epoch, and also the one that introduces the
most technical debt. The seed commit `90bf412` ("Coffee and changes.") is enormous
and heterogeneous. In a single commit it adds:

- the **reference documentation suite** (`ARCHITECTURE.md`,
  `BACKEND_INTEGRATION.md`, `FRONTEND_API.md`, `IMPLEMENTATION_STATUS.md`,
  `IMPLEMENTATION_SUMMARY.md`, `QUICKSTART.md` — together roughly 3,500 lines);
- the root **`docker-compose.yml`** that builds DB and back end together;
- a new **`RosBridgeService.cs`** (315 lines) — an `IHostedService` that maintains a
  `ClientWebSocket` to a ROS task-orchestrator endpoint (`RosWebSocket:Url`, default
  `ws://localhost:8080`) with a bounded reconnect loop (max 10 attempts, 5-second
  delay);
- a large block of **swarm-control methods on `RobotHub`** and their declarations on
  `IRealtimeService`: `SpawnRobots`, `SpawnRobotsWithIds`, `DeleteRobots`,
  `StartFollowLeader`, `StartFormation`, `StartTransport`, `StopTask`,
  `EmergencyStop`, `ControlLeader`, `SpawnObstacles`, `GetRosConnectionStatus`, and
  the bridge-forwarding methods `ForwardSwarmStatus`/`ForwardFleetEvent`. Each
  control method serialises a command and broadcasts it as a `SwarmCommand` (with
  task transitions emitted as `TaskEvent` and `EmergencyStop` envelopes);
- a front-end swarm stack authored in **TypeScript** (`SwarmControlPanel.tsx`,
  `useSwarmControl.ts`, `SwarmService.ts`) *alongside* a 970-line JavaScript
  `SwarmControlPanel.js` and a new `SwarmControl` app/config.

The very next commit, `d8fa711` ("Fixed build errors"), reveals that the TypeScript
files could not build in this JavaScript-toolchain project and **converts them to
JavaScript** (`.tsx → .js`, `.ts → .js`), deleting the 459-line `SwarmService.ts` in
favour of a 259-line `SwarmService.js`. The net result is that the repository now
carries *both* the original TypeScript artefacts (under `src/components`,
`src/hooks`, `src/services`) and their JavaScript descendants, plus the
GeeTS-module copies — a three-way duplication that §5 returns to.

The remainder of the epoch stabilises this large drop: `0181d2c` fixes CORS;
`7cd21f9` adds the `AddRobotNamespace` migration; `f8e17ba` ("Mashup swarm
controller and realtime") merges the standalone swarm panel into the Realtime UI,
substantially reworking `CommandPanel`, `CommandStatus`, and
`RealtimeConfigListImproved`; `e5c0198` and `e7d6fa0` are small swarm-panel and
connection fixes; and three commits (`aa10f8c`, `be5aa59`, `e1d82d1`) — these
carrying Claude co-authorship — resolve a layout regression by first imposing and
then *removing* a fixed `maxHeight`, ultimately deferring overflow handling to the
Fuse layout's `innerScroll`. The epoch's final functional change, `e1d82d1`,
hardens the UI against an inconsistent control surface by disabling every swarm
action button (deploy, delete, task start/stop, emergency stop) unless **both** the
server and the ROS bridge report connected.

### 3.8 Epoch VIII — Documentation and tooling (Jan / Mar / Jun)

Documentation was not a single event but a thread running through the later
history: the first `CLAUDE.md` in Epoch VI (`4a980de`), the large reference suite in
Epoch VII (`90bf412`), and finally a dedicated documentation pass on the working
branch. The two most recent substantive commits, `874b4a8` and `f136908`, revise
`CLAUDE.md` to document previously-undescribed components and then to retract
references (ESP32 firmware, the HeRo platform) deemed out of scope — a small but
instructive example of documentation being corrected to match the maintainers'
mental model of what the project *is*.

---

## 4. Quantitative Analysis

### 4.1 Authorship

Across the 54 non-merge post-baseline commits, authorship divides as follows:

| Author | Commits | Role in the narrative |
|--------|--------:|-----------------------|
| Félix Alejandro Guzmán | 41 | Primary developer; owns Epochs I–VI and parts of VII |
| Anyelo (Alvarez) | 9 + 1 | Drives the swarm-control epoch (VII) and integration/merges |
| Claude | 3 | Documentation commits on the working branch |

The two human contributors exhibit distinguishable styles: Guzmán's commits are
numerous, fine-grained, and increasingly adopt conventional-commit messages; the
Anyelo-authored commits are fewer, larger, and more integrative (the three merges
and the monolithic `90bf412` are his). Three merge commits join their parallel
lines of work.

### 4.2 Commit cadence and temporal clustering

The history is markedly *bursty* rather than steady. Activity clusters into a small
number of intense days separated by multi-month silences:

- **May–Jun 2025** — the ROS-bridge debugging burst, then quiet.
- **14–24 Oct 2025** — real-time reliability and the task subsystem.
- **31 Oct & 4 Nov 2025** — dashboard/detail UX and account RBAC.
- **28 Jan 2026** — the sensor pipeline overhaul.
- **10 Mar 2026** — the entire swarm-control epoch in a single day (≈11 commits).
- **29 Jun 2026** — documentation.

The concentration of Epoch VII into one calendar day, including a 970-line component
and a 315-line hosted service, is the clearest signal of a "big bang" integration
rather than incremental delivery — and, as §5 notes, the defects that survived into
that drop are consistent with its compressed timeline.

### 4.3 File churn and architectural hotspots

Ranking files by total churn (lines added + deleted across the post-baseline
history, excluding generated and binary files) identifies the system's centres of
gravity:

| Rank | File | Churn | Interpretation |
|-----:|------|------:|----------------|
| 1 | `Realtime/SwarmControlPanel.js` | 1,252 | The swarm UI; reworked repeatedly in Epoch VII |
| 2 | `behaviors/formation_control.py` | 1,249 | ROS swarm behaviour |
| 3 | `behaviors/collaborative_transport.py` | 1,003 | ROS swarm behaviour |
| 4 | `src/services/SwarmService.ts` | 918 | The TS service later superseded by `.js` |
| 5 | `behaviors/follow_leader.py` | 847 | ROS swarm behaviour |
| 6–8 | `ARCHITECTURE.md`, `FRONTEND_API.md`, `BACKEND_INTEGRATION.md` | 813 / 780 / 637 | The reference suite |
| — | `Realtime/RealtimeConfigListImproved.js` | 607 | The "improved" Realtime fork |
| — | `Realtime/RealtimeConfigList.js` | 500 | The *original* Realtime list, still evolving |
| — | `Dashboard/booths/RobotWidget.js` | 342 | The most-revised dashboard component |
| — | `Services/RosBridgeService.cs` | 315 | Authored once; see §5 |

Two observations follow. First, the swarm-control plane (UI plus ROS behaviours)
dominates the churn, confirming Epoch VII as the project's most significant single
investment. Second, the simultaneous presence of `RealtimeConfigList.js` (500) and
`RealtimeConfigListImproved.js` (607) high in the ranking quantifies the cost of the
fork-rather-than-replace decision taken in Epoch III: both files received
substantial ongoing maintenance.

---

## 5. Technical Debt and Observations

The history surfaces several pieces of debt that are invisible from any single
commit but evident across the whole record. They are recorded here not as criticism
but as a map for future maintenance.

**(O-1) A hosted service that is never hosted.** `RosBridgeService.cs` is written as
an `IHostedService` with full start/stop and reconnect logic, yet a search of the
composition root finds **no `AddHostedService<RosBridgeService>()` registration** in
`Program.cs`. As authored, the service is therefore dead code: the back-end-to-ROS
WebSocket bridge it implements is never started. Either the registration was lost in
the compressed Epoch VII drop, or the design pivoted to the SignalR bridge and the
WebSocket variant was abandoned mid-flight. This should be the first item any
maintainer reconciles, because the documentation added in the same commit describes
a bridge that does not run.

**(O-2) Three-way duplication of the swarm front end.** The same swarm-control
capability exists as (i) the original TypeScript trio under `src/components`,
`src/hooks`, `src/services`; (ii) their JavaScript conversions from `d8fa711`; and
(iii) the GeeTS-module `SwarmControlPanel.js`. Only one path is live. The dead paths
inflate churn metrics and present a hazard to anyone who edits the wrong copy.

**(O-3) The Realtime list fork.** `RealtimeConfigList.js` and
`RealtimeConfigListImproved.js` coexist, both received feature work after the fork,
and the "improved" variant became the de-facto target. Consolidating to one would
remove a standing source of divergence.

**(O-4) Accidental `node_modules` artefacts.** Commit `90bf412` committed
`node_modules/.bin/*` shim scripts (`.cmd`/`.ps1`) into version control. These are
build noise and should be removed and ignored.

**(O-5) Repository hygiene at the baseline.** The graft baseline carries the full
Fuse template's demo asset inventory (hundreds of unrelated showcase images) and a
17,693-line `prompts.txt`. Neither is load-bearing for the robotics application, and
both inflate clone size.

**(O-6) Authorisation by `null` convention.** The RBAC design (§3.5) encodes
"trusted internal caller" as a `null` role argument. This works but is fragile: any
new call site that forgets to pass a role will silently receive admin-equivalent,
unfiltered results. A dedicated internal-service path would be safer than an
in-band sentinel.

**(O-7) Recurring loose-vs-strong typing friction.** The sensor value type was
adjusted at least three times (`f81456a`, `12b750d`, `26504d6`) before the batch
overhaul (`e9aad9a`) settled the contract. The lesson for future ROS↔C# surfaces is
to fix the wire schema once, explicitly, rather than coercing at each call site.

---

## 6. Conclusion

Read end to end, the RobotSwarm history is the story of a generic admin-template
application being progressively specialised into a purpose-built swarm-control
platform. The trajectory is legible: first the real-time *transport* was made
reliable (Epochs I–II), then the *task* and *presentation* layers were built on top
of it (Epochs III–IV), then *authorisation* was retrofitted (Epoch V), then the
*data pipeline* was rationalised (Epoch VI), and finally an ambitious
*swarm-control plane* was added in a single large push (Epoch VII) whose
integration defects (O-1, O-2) remain partly unreconciled. Documentation (Epoch
VIII) trailed the code throughout, catching up only at the end.

The dominant engineering virtue on display is iterative empiricism — most visibly in
the Epoch I bridge-debugging sequence and the Epoch II rate-limiting refinement —
and the dominant risk is big-bang integration, most visibly in Epoch VII. A
maintainer inheriting this codebase should prioritise reconciling the swarm-control
plane: register or retire `RosBridgeService`, collapse the duplicated front-end
implementations to a single source of truth, and align the reference documentation
with whichever bridge actually runs. With those reconciliations, the platform's
underlying architecture — a SignalR star with a strongly-typed persistent core and a
rate-limited ROS bridge — is sound and well-positioned for further work.

---

## Appendix A — Complete Commit Ledger

Chronological, oldest first. Merge commits are marked **[M]**. The grafted baseline
is marked **[B]**.

| # | Hash | Date | Author | Subject |
|--:|------|------|--------|---------|
| 1 | `7d6af26` | 2025-05-13 | Anyelo Alvarez | **[B][M]** Merge baseline import |
| 2 | `f81456a` | 2025-05-13 | Guzmán | Allow non-float sensor values |
| 3 | `12b750d` | 2025-05-16 | Anyelo Alvarez | str to int |
| 4 | `14234fe` | 2025-05-16 | Anyelo Alvarez | **[M]** Merge main |
| 5 | `8923a93` | 2025-05-16 | Guzmán | Inject hub context into TaskLogService |
| 6 | `e472d1e` | 2025-05-16 | Guzmán | Log commands via rospy |
| 7 | `1fc0cd2` | 2025-05-16 | Guzmán | Log command content; notify targets |
| 8 | `140b887` | 2025-05-16 | Guzmán | Accept multiple command arguments |
| 9 | `4514f1c` | 2025-05-16 | Guzmán | Fix multi-arg command extraction |
| 10 | `29891f3` | 2025-05-16 | Guzmán | Fix robot-ID key in command handling |
| 11 | `216511f` | 2025-06-05 | Guzmán | Add robot status to LeafSorting form |
| 12 | `51dce1e` | 2025-06-05 | Guzmán | Notify robots-available on update |
| 13 | `9049ccc` | 2025-10-14 | Guzmán | CI/compose log maintenance; mise.toml |
| 14 | `ee5bf24` | 2025-10-14 | Guzmán | Type-specific rate limiting; sensor format |
| 15 | `abf11d2` | 2025-10-17 | Anyelo | Fix robot topics |
| 16 | `a10c98a` | 2025-10-17 | Anyelo | **[M]** Merge main |
| 17 | `b284657` | 2025-10-17 | Anyelo | Dummy-bot fixes |
| 18 | `4f8fbe9` | 2025-10-17 | Anyelo | Dummy-bot fixes (cont.) |
| 19 | `2407322` | 2025-10-17 | Guzmán | Promise-based SignalR readiness |
| 20 | `26504d6` | 2025-10-17 | Guzmán | Convert sensor value to string |
| 21 | `a3461ab` | 2025-10-24 | Guzmán | Sort sensor readings by notes |
| 22 | `a6ebf34` | 2025-10-24 | Guzmán | Remove debug logs |
| 23 | `4ed6512` | 2025-10-24 | Guzmán | Task-send UI (RealtimeConfigList) |
| 24 | `f309043` | 2025-10-24 | Guzmán | Task Logs feature (UI + API) |
| 25 | `aacfd8d` | 2025-10-24 | Guzmán | Drop api/ prefix from TaskLog route |
| 26 | `523e86d` | 2025-10-24 | Guzmán | Cancel tasks by robot + account |
| 27 | `4d69a60` | 2025-10-24 | Guzmán | Account-based cancel authorization |
| 28 | `8a86c73` | 2025-10-31 | Guzmán | Dashboard grid + RobotWidget rework |
| 29 | `2519acf` | 2025-10-31 | Guzmán | GetByRobot task-log endpoint |
| 30 | `9960c5e` | 2025-10-31 | Guzmán | Robot-detail page |
| 31 | `33b6899` | 2025-10-31 | Guzmán | Route tweak (dashboard) |
| 32 | `7fd5538` | 2025-10-31 | Guzmán | CommandPanel + config-app UI refactor |
| 33 | `c5344f1` | 2025-10-31 | Guzmán | Header relabel "TAREAS" |
| 34 | `aba74da` | 2025-11-04 | Guzmán | Account RBAC |
| 35 | `790646c` | 2025-11-04 | Guzmán | Allow null-role internal GetAll |
| 36 | `4696f28` | 2025-11-04 | Guzmán | Remove account params (frontend) |
| 37 | `3d112cf` | 2025-11-04 | Guzmán | Fix account endpoint |
| 38 | `e915d60` | 2025-11-04 | Guzmán | Add auth headers to getAccounts |
| 39 | `6ee63b4` | 2025-11-04 | Guzmán | Remove .data wrapper |
| 40 | `e188b41` | 2025-11-04 | Guzmán | Adapt Accounts app for GTS roles |
| 41 | `4a980de` | 2026-01-28 | Guzmán | Add initial CLAUDE.md |
| 42 | `44c7bb3` | 2026-01-28 | Guzmán | Realtime layout: command beside VNC |
| 43 | `e9aad9a` | 2026-01-28 | Guzmán | Sensor batch pipeline overhaul |
| 44 | `23f3015` | 2026-01-28 | Guzmán | Docker: dotnet restore for migrations |
| 45 | `90bf412` | 2026-03-10 | Anyelo | Swarm-control plane + docs (large) |
| 46 | `d8fa711` | 2026-03-10 | Anyelo | Convert swarm TS → JS (build fix) |
| 47 | `0181d2c` | 2026-03-10 | Anyelo | Fix CORS |
| 48 | `7cd21f9` | 2026-03-10 | Anyelo | AddRobotNamespace migration |
| 49 | `f8e17ba` | 2026-03-10 | Anyelo | Merge swarm controller into Realtime |
| 50 | `e5c0198` | 2026-03-10 | Anyelo | Swarm-panel changes |
| 51 | `e7d6fa0` | 2026-03-10 | Guzmán | Fix connection issue |
| 52 | `677d095` | 2026-03-10 | Anyelo | **[M]** Merge main |
| 53 | `aa10f8c` | 2026-03-10 | Guzmán | Fix realtime scroll/overflow |
| 54 | `be5aa59` | 2026-03-10 | Guzmán | Defer overflow to Fuse innerScroll |
| 55 | `e1d82d1` | 2026-03-10 | Guzmán | Disable swarm buttons when ROS down |
| 56 | `874b4a8` | 2026-06-29 | Claude | CLAUDE.md: add missing components |
| 57 | `f136908` | 2026-06-29 | Claude | CLAUDE.md: drop firmware/HeRo refs |

*(Commit ordering within a single day reflects the topological/commit-date order
reported by git; exact intra-day sequence may vary for commits sharing a date.)*

## Appendix B — Methodological Notes

- **Tooling.** Chronology from `git log --graph --all`; per-commit statistics from
  `git show --numstat`; close reading from `git show <hash> -- <path>`.
- **Noise exclusion.** `package-lock.json`, binary assets
  (`*.png|jpg|jpeg|ico|woff2|ttf|svg|gif`), EF `*.Designer.cs` files, `prompts.txt`,
  and `node_modules/**` were filtered from churn and diff analysis to isolate
  engineering signal.
- **Churn metric.** "Churn" denotes added + deleted lines summed over the
  post-baseline range `7d6af26..HEAD`; it measures *activity*, not quality, and a
  high score may indicate either heavy investment or repeated rework.
- **Reproducibility.** Every claim in §3 is traceable to a named commit hash; every
  figure in §4 derives from the commands recorded above and can be re-run against
  the same range.
