using System.Text.Json;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Infrastructure;

namespace SwarmWorker.Runtime;

public sealed record ManagedSessionInfo(
    Guid SessionId,
    string ContainerId,
    string ContainerName,
    string Image,
    bool Running,
    string State,
    IReadOnlyDictionary<string, string> Labels);

public sealed record SessionOperationResult(
    Guid SessionId,
    string? ContainerId,
    string? ContainerName,
    string Image,
    IReadOnlyList<string> RobotIds,
    bool Created);

public sealed record FailSafeOperationResult(
    Guid SessionId,
    bool EmergencyStopConfirmed,
    bool ContainerStopped,
    string Reason);

public sealed class DockerSessionManager
{
    private const string ReadRosterScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        + " && exec rostopic echo -n 1 /fleet/robot_list";

    private const string PublishSpawnScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        + " && exec rostopic pub -1 /fleet/spawn_command std_msgs/String \"$1\"";

    private const string PublishDeleteScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        + " && exec rostopic pub -1 /fleet/delete_command std_msgs/String \"$1\"";

    private const string PublishSwarmCommandScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        + " && exec rostopic pub -1 /swarm/commands std_msgs/String \"$1\"";

    private const string PublishControlHeartbeatScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        + " && exec rostopic pub -1 /swarm/control_heartbeat std_msgs/Empty '{}'";

    private const string ReadSwarmStatusScript =
        "source /opt/ros/noetic/setup.bash"
        + " && source /catkin_ws/devel/setup.bash"
        // rostopic folds large String messages across YAML lines. Read the
        // message field directly so the status parsers receive one full line.
        + " && exec python3 -c 'import json, rospy; "
        + "from std_msgs.msg import String; "
        + "rospy.init_node(\"swarm_worker_status_reader\", "
        + "anonymous=True, disable_signals=True); "
        + "message = rospy.wait_for_message("
        + "\"/swarm/status\", String, timeout=5.0); "
        + "print(\"data: \" + json.dumps(message.data))'";

    private readonly IDockerCli _docker;
    private readonly WorkerOptions _options;
    private readonly ILogger<DockerSessionManager> _logger;

    public DockerSessionManager(
        IDockerCli docker,
        IOptions<WorkerOptions> options,
        ILogger<DockerSessionManager> logger)
    {
        _docker = docker;
        _options = options.Value;
        _logger = logger;
    }

    public async Task ReconcileAsync(CancellationToken cancellationToken)
    {
        var sessions = await GetManagedSessionsAsync(cancellationToken);
        foreach (var duplicate in sessions.GroupBy(session => session.SessionId)
                     .Where(group => group.Count() > 1))
        {
            _logger.LogCritical(
                "Session {SessionId} has {ContainerCount} managed containers; commands will be rejected until reconciled.",
                duplicate.Key,
                duplicate.Count());
        }

        _logger.LogInformation(
            "Docker reconciliation found {RunningCount} running and {StoppedCount} stopped managed sessions.",
            sessions.Count(session => session.Running),
            sessions.Count(session => !session.Running));
    }

    public async Task<IReadOnlyList<ManagedSessionInfo>> GetManagedSessionsAsync(
        CancellationToken cancellationToken)
    {
        var listResult = await _docker.RunAsync(
            new[]
            {
                "ps",
                "-a",
                "--filter",
                $"label={SessionLabels.Managed}=true",
                "--filter",
                $"label={SessionLabels.WorkerId}={_options.WorkerId:D}",
                "--format",
                "{{.ID}}"
            },
            cancellationToken);

        if (!listResult.IsSuccess)
        {
            throw new DockerCliException("list managed containers", listResult);
        }

        var containerIds = listResult.StandardOutput
            .Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

        var sessions = new List<ManagedSessionInfo>();
        foreach (var containerId in containerIds)
        {
            var inspection = await InspectContainerAsync(containerId, cancellationToken);
            if (inspection is not null)
            {
                sessions.Add(inspection);
            }
        }

        return sessions;
    }

    public async Task<SessionOperationResult> ProvisionAsync(
        Guid sessionId,
        FleetCommandPayload payload,
        CancellationToken cancellationToken)
    {
        var existing = await FindSessionContainersAsync(sessionId, cancellationToken);
        if (existing.Count > 1)
        {
            throw new InvalidOperationException(
                $"Session {sessionId} has multiple managed containers.");
        }

        var created = false;
        ManagedSessionInfo session;
        if (existing.Count == 1)
        {
            session = existing[0];
            ValidateOwnership(session);
            if (!SpecificationMatches(session, payload.ArenaVersion))
            {
                if (session.Running)
                {
                    throw new InvalidOperationException(
                        $"Running session {session.SessionId} does not match the configured ROS image or launch limits.");
                }

                _logger.LogInformation(
                    "Removing stale stopped container {ContainerId} for session {SessionId}.",
                    session.ContainerId,
                    session.SessionId);
                await RunRequiredAsync(
                    "remove stale session container",
                    new[] { "rm", "--force", session.ContainerId },
                    cancellationToken);
                existing = Array.Empty<ManagedSessionInfo>();
            }
        }

        if (existing.Count == 1)
        {
            session = existing[0];
            if (!session.Running)
            {
                await RunRequiredAsync(
                    "start session container",
                    new[] { "start", session.ContainerId },
                    cancellationToken);
                session = session with { Running = true, State = "running" };
            }
        }
        else
        {
            var runningCount = (await GetManagedSessionsAsync(cancellationToken))
                .Count(candidate => candidate.Running);
            if (runningCount >= _options.MaxConcurrentSessions)
            {
                throw new InvalidOperationException(
                    $"Worker capacity of {_options.MaxConcurrentSessions} sessions has been reached.");
            }

            await EnsureSessionNetworkAsync(sessionId, cancellationToken);

            var specification = new SessionContainerSpec(
                _options.WorkerId,
                sessionId,
                payload.ArenaVersion,
                _options.SessionImage,
                _options.ImageVersion,
                _options.MaxRobotsPerSession);

            var createResult = await _docker.RunAsync(
                DockerArgumentBuilder.BuildCreateContainer(specification, _options),
                cancellationToken,
                TimeSpan.FromMinutes(2));
            if (!createResult.IsSuccess)
            {
                throw new DockerCliException("create session container", createResult);
            }

            var containerId = createResult.StandardOutput.Trim();
            if (string.IsNullOrWhiteSpace(containerId))
            {
                throw new InvalidOperationException("Docker did not return a container ID.");
            }

            await RunRequiredAsync(
                "start session container",
                new[] { "start", containerId },
                cancellationToken);

            session = await InspectContainerAsync(containerId, cancellationToken)
                ?? throw new InvalidOperationException("Created session container disappeared.");
            created = true;
        }

        await WaitForRosterAsync(
            session.ContainerId,
            expectedRobotIds: null,
            TimeSpan.FromSeconds(_options.RosReadyTimeoutSeconds),
            cancellationToken);

        var robotIds = await ReplaceFleetAsync(
            session.ContainerId,
            payload,
            cancellationToken);

        return new SessionOperationResult(
            sessionId,
            session.ContainerId,
            session.ContainerName,
            session.Image,
            robotIds,
            created);
    }

    public async Task<SessionOperationResult> UpdateFleetAsync(
        Guid sessionId,
        FleetCommandPayload payload,
        CancellationToken cancellationToken)
    {
        var session = await GetSingleRunningSessionAsync(sessionId, cancellationToken);
        var robotIds = await ReplaceFleetAsync(
            session.ContainerId,
            payload,
            cancellationToken);

        return new SessionOperationResult(
            sessionId,
            session.ContainerId,
            session.ContainerName,
            session.Image,
            robotIds,
            Created: false);
    }

    public async Task<SessionOperationResult> StopAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var existing = await FindSessionContainersAsync(sessionId, cancellationToken);
        if (existing.Count > 1)
        {
            throw new InvalidOperationException(
                $"Session {sessionId} has multiple managed containers.");
        }

        if (existing.Count == 1)
        {
            var session = existing[0];
            ValidateOwnership(session);

            if (session.Running)
            {
                await RunRequiredAsync(
                    "stop session container",
                    new[] { "stop", "--time", "30", session.ContainerId },
                    cancellationToken,
                    TimeSpan.FromSeconds(45));
            }

            await RunRequiredAsync(
                "remove session container",
                new[] { "rm", "--force", session.ContainerId },
                cancellationToken);
        }

        await RemoveSessionNetworkAsync(sessionId, cancellationToken);

        return new SessionOperationResult(
            sessionId,
            null,
            null,
            _options.SessionImage,
            Array.Empty<string>(),
            Created: false);
    }

    public async Task PublishSwarmCommandAsync(
        Guid sessionId,
        JsonElement command,
        CancellationToken cancellationToken)
    {
        var session = await GetSingleRunningSessionAsync(sessionId, cancellationToken);
        await PublishSwarmCommandAsync(session, command, cancellationToken);
    }

    public async Task<FailSafeOperationResult> ApplyDisconnectFailSafeAsync(
        ManagedSessionInfo session,
        CancellationToken cancellationToken)
    {
        ValidateOwnership(session);
        const string reason =
            "Backend contact was lost, so the worker applied its local safety stop.";
        var command = JsonSerializer.SerializeToElement(new
        {
            command = "emergency_stop",
            parameters = new { }
        });

        try
        {
            await PublishSwarmCommandAsync(session, command, cancellationToken);
            await WaitForEmergencyStopAsync(
                session,
                expectedState: true,
                cancellationToken);
            return new FailSafeOperationResult(
                session.SessionId,
                EmergencyStopConfirmed: true,
                ContainerStopped: false,
                reason);
        }
        catch (Exception exception)
            when (exception is DockerCliException
                  or TimeoutException
                  or FormatException
                  or InvalidOperationException)
        {
            _logger.LogError(
                exception,
                "ROS did not acknowledge the disconnect fail-safe for session {SessionId}; stopping its container.",
                session.SessionId);
            await RunRequiredAsync(
                "stop session container after fail-safe timeout",
                new[] { "stop", "--time", "10", session.ContainerId },
                cancellationToken,
                TimeSpan.FromSeconds(20));
            return new FailSafeOperationResult(
                session.SessionId,
                EmergencyStopConfirmed: false,
                ContainerStopped: true,
                reason + " ROS did not acknowledge it, so the session container was stopped.");
        }
    }

    public async Task PublishControlHeartbeatAsync(
        ManagedSessionInfo session,
        CancellationToken cancellationToken)
    {
        ValidateOwnership(session);
        if (!session.Running)
        {
            return;
        }

        var result = await _docker.RunAsync(
            new[]
            {
                "exec",
                session.ContainerId,
                "/bin/bash",
                "-lc",
                PublishControlHeartbeatScript
            },
            cancellationToken,
            TimeSpan.FromSeconds(5));
        if (!result.IsSuccess)
        {
            throw new DockerCliException("publish ROS control heartbeat", result);
        }
    }

    private async Task PublishSwarmCommandAsync(
        ManagedSessionInfo session,
        JsonElement command,
        CancellationToken cancellationToken)
    {
        var rosMessage = JsonSerializer.Serialize(new { data = command.GetRawText() });
        var result = await _docker.RunAsync(
            new[]
            {
                "exec",
                session.ContainerId,
                "/bin/bash",
                "-lc",
                PublishSwarmCommandScript,
                "swarm-worker",
                rosMessage
            },
            cancellationToken,
            TimeSpan.FromSeconds(30));

        if (!result.IsSuccess)
        {
            throw new DockerCliException("publish ROS swarm command", result);
        }
    }

    public async Task<RosTaskStatus?> ReadTaskStatusAsync(
        ManagedSessionInfo session,
        CancellationToken cancellationToken)
    {
        ValidateOwnership(session);
        if (!session.Running)
        {
            return null;
        }

        var result = await _docker.RunAsync(
            new[]
            {
                "exec",
                session.ContainerId,
                "/bin/bash",
                "-lc",
                ReadSwarmStatusScript
            },
            cancellationToken,
            TimeSpan.FromSeconds(8));

        if (!result.IsSuccess)
        {
            throw new DockerCliException("read ROS swarm status", result);
        }

        return TaskStatusParser.Parse(result.StandardOutput);
    }

    public async Task WaitForEmergencyStopAsync(
        Guid sessionId,
        bool expectedState,
        CancellationToken cancellationToken)
    {
        var session = await GetSingleRunningSessionAsync(sessionId, cancellationToken);
        await WaitForEmergencyStopAsync(session, expectedState, cancellationToken);
    }

    private async Task WaitForEmergencyStopAsync(
        ManagedSessionInfo session,
        bool expectedState,
        CancellationToken cancellationToken)
    {
        var deadline = DateTime.UtcNow
            + TimeSpan.FromSeconds(_options.EmergencyStopTimeoutSeconds);
        Exception? lastError = null;

        while (DateTime.UtcNow < deadline)
        {
            cancellationToken.ThrowIfCancellationRequested();
            try
            {
                var result = await _docker.RunAsync(
                    new[]
                    {
                        "exec",
                        session.ContainerId,
                        "/bin/bash",
                        "-lc",
                        ReadSwarmStatusScript
                    },
                    cancellationToken,
                    TimeSpan.FromSeconds(8));
                if (!result.IsSuccess)
                {
                    throw new DockerCliException("confirm ROS emergency-stop state", result);
                }

                if (EmergencyStopStatusParser.Parse(result.StandardOutput) == expectedState)
                {
                    return;
                }
            }
            catch (Exception exception)
                when (exception is DockerCliException
                      or FormatException
                      or TimeoutException)
            {
                lastError = exception;
            }

            await Task.Delay(TimeSpan.FromMilliseconds(200), cancellationToken);
        }

        throw new TimeoutException(
            expectedState
                ? "ROS did not confirm the emergency stop before the timeout."
                : "ROS did not confirm the emergency-stop reset before the timeout.",
            lastError);
    }

    private async Task<IReadOnlyList<string>> ReplaceFleetAsync(
        string containerId,
        FleetCommandPayload payload,
        CancellationToken cancellationToken)
    {
        var currentRoster = await ReadRosterAsync(containerId, cancellationToken);
        if (currentRoster.SequenceEqual(payload.RobotIds, StringComparer.Ordinal))
        {
            return currentRoster;
        }

        // The current ROS topic API cannot place incremental additions relative
        // to existing robots. Replacing the fleet is safer than spawning new
        // robots at potentially occupied grid coordinates.
        if (currentRoster.Count > 0)
        {
            await PublishFleetCommandAsync(
                containerId,
                PublishDeleteScript,
                new { all = true },
                cancellationToken);
            await WaitForRosterAsync(
                containerId,
                Array.Empty<string>(),
                TimeSpan.FromSeconds(_options.FleetUpdateTimeoutSeconds),
                cancellationToken);
        }

        await PublishFleetCommandAsync(
            containerId,
            PublishSpawnScript,
            new
            {
                count = payload.DesiredRobotCount,
                pattern = payload.SpawnPattern,
                robot_ids = payload.RobotIds
            },
            cancellationToken);

        return await WaitForRosterAsync(
            containerId,
            payload.RobotIds,
            TimeSpan.FromSeconds(_options.FleetUpdateTimeoutSeconds),
            cancellationToken);
    }

    private async Task PublishFleetCommandAsync(
        string containerId,
        string fixedScript,
        object command,
        CancellationToken cancellationToken)
    {
        var commandJson = JsonSerializer.Serialize(command);
        var rosMessage = JsonSerializer.Serialize(new { data = commandJson });

        var result = await _docker.RunAsync(
            new[]
            {
                "exec",
                containerId,
                "/bin/bash",
                "-lc",
                fixedScript,
                "swarm-worker",
                rosMessage
            },
            cancellationToken,
            TimeSpan.FromSeconds(30));

        if (!result.IsSuccess)
        {
            throw new DockerCliException("publish ROS fleet command", result);
        }
    }

    private async Task<IReadOnlyList<string>> WaitForRosterAsync(
        string containerId,
        IReadOnlyList<string>? expectedRobotIds,
        TimeSpan timeout,
        CancellationToken cancellationToken)
    {
        var deadline = DateTime.UtcNow + timeout;
        Exception? lastError = null;

        while (DateTime.UtcNow < deadline)
        {
            cancellationToken.ThrowIfCancellationRequested();
            try
            {
                var roster = await ReadRosterAsync(containerId, cancellationToken);
                if (expectedRobotIds is null
                    || roster.SequenceEqual(expectedRobotIds, StringComparer.Ordinal))
                {
                    return roster;
                }
            }
            catch (Exception exception)
                when (exception is DockerCliException
                      or FormatException
                      or TimeoutException)
            {
                lastError = exception;
            }

            await Task.Delay(TimeSpan.FromSeconds(2), cancellationToken);
        }

        throw new TimeoutException(
            expectedRobotIds is null
                ? "ROS fleet manager did not become ready before the timeout."
                : "ROS fleet did not converge to the requested robot roster before the timeout.",
            lastError);
    }

    private async Task<IReadOnlyList<string>> ReadRosterAsync(
        string containerId,
        CancellationToken cancellationToken)
    {
        var result = await _docker.RunAsync(
            new[]
            {
                "exec",
                containerId,
                "/bin/bash",
                "-lc",
                ReadRosterScript
            },
            cancellationToken,
            TimeSpan.FromSeconds(12));

        if (!result.IsSuccess)
        {
            throw new DockerCliException("read ROS fleet roster", result);
        }

        return FleetRosterParser.Parse(result.StandardOutput);
    }

    private async Task<ManagedSessionInfo> GetSingleRunningSessionAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var matches = await FindSessionContainersAsync(sessionId, cancellationToken);
        if (matches.Count != 1)
        {
            throw new InvalidOperationException(
                matches.Count == 0
                    ? $"Session {sessionId} has no managed container."
                    : $"Session {sessionId} has multiple managed containers.");
        }

        var session = matches[0];
        ValidateOwnership(session);
        if (!session.Running)
        {
            throw new InvalidOperationException($"Session {sessionId} container is not running.");
        }

        return session;
    }

    private async Task<IReadOnlyList<ManagedSessionInfo>> FindSessionContainersAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var result = await _docker.RunAsync(
            new[]
            {
                "ps",
                "-a",
                "--filter",
                $"label={SessionLabels.Managed}=true",
                "--filter",
                $"label={SessionLabels.SessionId}={sessionId:D}",
                "--format",
                "{{.ID}}"
            },
            cancellationToken);

        if (!result.IsSuccess)
        {
            throw new DockerCliException("find session container", result);
        }

        var matches = new List<ManagedSessionInfo>();
        foreach (var containerId in result.StandardOutput.Split(
                     '\n',
                     StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            var inspection = await InspectContainerAsync(containerId, cancellationToken);
            if (inspection is not null)
            {
                matches.Add(inspection);
            }
        }

        return matches;
    }

    private async Task<ManagedSessionInfo?> InspectContainerAsync(
        string containerId,
        CancellationToken cancellationToken)
    {
        var result = await _docker.RunAsync(
            new[] { "inspect", containerId },
            cancellationToken);
        if (!result.IsSuccess)
        {
            _logger.LogWarning(
                "Managed container {ContainerId} disappeared during reconciliation.",
                containerId);
            return null;
        }

        using var document = JsonDocument.Parse(result.StandardOutput);
        var root = document.RootElement.EnumerateArray().Single();
        var labelsElement = root.GetProperty("Config").GetProperty("Labels");
        var labels = labelsElement.ValueKind == JsonValueKind.Object
            ? labelsElement.EnumerateObject().ToDictionary(
                property => property.Name,
                property => property.Value.GetString() ?? string.Empty,
                StringComparer.Ordinal)
            : new Dictionary<string, string>(StringComparer.Ordinal);

        if (!labels.TryGetValue(SessionLabels.SessionId, out var sessionIdText)
            || !Guid.TryParse(sessionIdText, out var sessionId))
        {
            _logger.LogError(
                "Managed container {ContainerId} has no valid session label.",
                containerId);
            return null;
        }

        var state = root.GetProperty("State");
        return new ManagedSessionInfo(
            sessionId,
            root.GetProperty("Id").GetString() ?? containerId,
            (root.GetProperty("Name").GetString() ?? string.Empty).TrimStart('/'),
            root.GetProperty("Config").GetProperty("Image").GetString() ?? string.Empty,
            state.GetProperty("Running").GetBoolean(),
            state.GetProperty("Status").GetString() ?? "unknown",
            labels);
    }

    private bool SpecificationMatches(
        ManagedSessionInfo session,
        string arenaVersion)
    {
        if (!session.Image.Equals(_options.SessionImage, StringComparison.Ordinal))
        {
            return false;
        }

        if (!session.Labels.TryGetValue(SessionLabels.ArenaVersion, out var actualArena)
            || !actualArena.Equals(arenaVersion, StringComparison.Ordinal))
        {
            return false;
        }

        var expectedImageVersion = _options.ImageVersion ?? string.Empty;
        if (!session.Labels.TryGetValue(SessionLabels.ImageVersion, out var actualImageVersion)
            || !actualImageVersion.Equals(expectedImageVersion, StringComparison.Ordinal))
        {
            return false;
        }

        return session.Labels.TryGetValue(SessionLabels.MaxRobots, out var maxRobots)
            && int.TryParse(maxRobots, out var parsedMaxRobots)
            && parsedMaxRobots == _options.MaxRobotsPerSession;
    }

    private void ValidateOwnership(ManagedSessionInfo session)
    {
        if (!session.Labels.TryGetValue(SessionLabels.WorkerId, out var workerId)
            || !Guid.TryParse(workerId, out var parsedWorkerId)
            || parsedWorkerId != _options.WorkerId)
        {
            throw new InvalidOperationException(
                $"Session {session.SessionId} is owned by a different worker.");
        }
    }

    private async Task EnsureSessionNetworkAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var name = SessionResourceNames.Network(sessionId);
        var inspect = await _docker.RunAsync(
            new[] { "network", "inspect", name },
            cancellationToken);

        if (inspect.IsSuccess)
        {
            ValidateNetworkOwnership(inspect.StandardOutput, sessionId);
            return;
        }

        var create = await _docker.RunAsync(
            DockerArgumentBuilder.BuildCreateNetwork(_options.WorkerId, sessionId),
            cancellationToken);
        if (!create.IsSuccess)
        {
            var retryInspect = await _docker.RunAsync(
                new[] { "network", "inspect", name },
                cancellationToken);
            if (!retryInspect.IsSuccess)
            {
                throw new DockerCliException("create session network", create);
            }

            ValidateNetworkOwnership(retryInspect.StandardOutput, sessionId);
        }
    }

    private async Task RemoveSessionNetworkAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var name = SessionResourceNames.Network(sessionId);
        var inspect = await _docker.RunAsync(
            new[] { "network", "inspect", name },
            cancellationToken);
        if (!inspect.IsSuccess)
        {
            return;
        }

        ValidateNetworkOwnership(inspect.StandardOutput, sessionId);
        var remove = await _docker.RunAsync(
            new[] { "network", "rm", name },
            cancellationToken);
        if (!remove.IsSuccess)
        {
            throw new DockerCliException("remove session network", remove);
        }
    }

    private void ValidateNetworkOwnership(string inspectionJson, Guid sessionId)
    {
        using var document = JsonDocument.Parse(inspectionJson);
        var root = document.RootElement.EnumerateArray().Single();
        var labels = root.GetProperty("Labels");

        var workerLabel = labels.TryGetProperty(SessionLabels.WorkerId, out var worker)
            ? worker.GetString()
            : null;
        var sessionLabel = labels.TryGetProperty(SessionLabels.SessionId, out var session)
            ? session.GetString()
            : null;

        if (!Guid.TryParse(workerLabel, out var workerId)
            || workerId != _options.WorkerId
            || !Guid.TryParse(sessionLabel, out var parsedSessionId)
            || parsedSessionId != sessionId)
        {
            throw new InvalidOperationException(
                $"Docker network '{SessionResourceNames.Network(sessionId)}' is not owned by this session.");
        }
    }

    private async Task RunRequiredAsync(
        string operation,
        IReadOnlyList<string> arguments,
        CancellationToken cancellationToken,
        TimeSpan? timeout = null)
    {
        var result = await _docker.RunAsync(arguments, cancellationToken, timeout);
        if (!result.IsSuccess)
        {
            throw new DockerCliException(operation, result);
        }
    }
}
