using System.Text.Json;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class DockerSessionManagerTests
{
    [Fact]
    public async Task PublishesControlHeartbeatThroughTheFixedRosTopic()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty));
        var manager = new DockerSessionManager(
            docker,
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://backend.example.test",
                WorkerId = workerId,
                WorkerSecret = new string('a', 32),
                Name = "test-worker",
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        await manager.PublishControlHeartbeatAsync(
            session,
            CancellationToken.None);

        var arguments = Assert.Single(docker.Calls);
        Assert.Equal("exec", arguments[0]);
        Assert.Contains(
            "rostopic pub -1 /swarm/control_heartbeat std_msgs/Empty '{}'",
            arguments[4]);
    }

    [Fact]
    public async Task ReadsSwarmStatusWithoutRosYamlFolding()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var statusJson = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = taskRunId,
                status = "running",
                progress = 0.25
            },
            robots = Enumerable.Range(0, 20).Select(index => new
            {
                id = $"tb3_{index}",
                ranges = Enumerable.Repeat(3.5, 360)
            })
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(
                0,
                $"data: {JsonSerializer.Serialize(statusJson)}\n",
                string.Empty));
        var manager = new DockerSessionManager(
            docker,
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://backend.example.test",
                WorkerId = workerId,
                WorkerSecret = new string('a', 32),
                Name = "test-worker",
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        var status = await manager.ReadTaskStatusAsync(
            session,
            CancellationToken.None);

        Assert.NotNull(status);
        Assert.Equal(taskRunId, status.TaskRunId);
        Assert.Equal("Running", status.State);
        Assert.Equal(0.25, status.Progress);

        var arguments = Assert.Single(docker.Calls);
        Assert.Contains("rospy.wait_for_message", arguments[4]);
        Assert.Contains("timeout=5.0", arguments[4]);
        Assert.Contains("json.dumps(message.data)", arguments[4]);
        Assert.DoesNotContain(
            "rostopic echo -n 1 /swarm/status",
            arguments[4]);
    }

    [Fact]
    public async Task PublishesSwarmJsonAsASeparateDockerArgument()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty));
        var options = new WorkerOptions
        {
            BackendUrl = "https://backend.example.test",
            WorkerId = workerId,
            WorkerSecret = new string('a', 32),
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test"
        };
        var manager = new DockerSessionManager(
            docker,
            Options.Create(options),
            NullLogger<DockerSessionManager>.Instance);
        var command = JsonSerializer.SerializeToElement(new
        {
            command = "start_task",
            parameters = new
            {
                task_id = Guid.NewGuid(),
                label = "\"; touch /tmp/should-not-run; #"
            }
        });

        await manager.PublishSwarmCommandAsync(
            sessionId,
            command,
            CancellationToken.None);

        var publishArguments = docker.Calls[2];
        Assert.Equal("exec", publishArguments[0]);
        Assert.Equal("container-1", publishArguments[1]);
        Assert.Contains(
            "rostopic pub -1 /swarm/commands std_msgs/String \"$1\"",
            publishArguments[4]);
        Assert.DoesNotContain("touch /tmp/should-not-run", publishArguments[4]);
        Assert.Contains("touch /tmp/should-not-run", publishArguments[6]);
    }

    private static string BuildInspection(Guid workerId, Guid sessionId)
    {
        return JsonSerializer.Serialize(new[]
        {
            new
            {
                Id = "container-1",
                Name = $"/{SessionResourceNames.Container(sessionId)}",
                Config = new
                {
                    Image = "robotswarm/ros-noetic:test",
                    Labels = new Dictionary<string, string>
                    {
                        [SessionLabels.Managed] = "true",
                        [SessionLabels.WorkerId] = workerId.ToString("D"),
                        [SessionLabels.SessionId] = sessionId.ToString("D")
                    }
                },
                State = new
                {
                    Running = true,
                    Status = "running"
                }
            }
        });
    }

    private sealed class RecordingDockerCli : IDockerCli
    {
        private readonly Queue<DockerCommandResult> _results;

        public RecordingDockerCli(params DockerCommandResult[] results)
        {
            _results = new Queue<DockerCommandResult>(results);
        }

        public List<IReadOnlyList<string>> Calls { get; } = new();

        public Task<DockerCommandResult> RunAsync(
            IReadOnlyList<string> arguments,
            CancellationToken cancellationToken,
            TimeSpan? timeout = null)
        {
            Calls.Add(arguments.ToArray());
            return Task.FromResult(_results.Dequeue());
        }
    }
}
