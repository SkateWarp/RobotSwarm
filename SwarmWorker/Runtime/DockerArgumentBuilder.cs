using System.Globalization;
using SwarmWorker.Configuration;

namespace SwarmWorker.Runtime;

public static class SessionLabels
{
    public const string Managed = "io.robotswarm.managed";
    public const string WorkerId = "io.robotswarm.worker-id";
    public const string SessionId = "io.robotswarm.session-id";
    public const string ArenaVersion = "io.robotswarm.arena-version";
    public const string ImageVersion = "io.robotswarm.image-version";
    public const string MaxRobots = "io.robotswarm.max-robots";
}

public sealed record SessionContainerSpec(
    Guid WorkerId,
    Guid SessionId,
    string ArenaVersion,
    string Image,
    string? ImageVersion,
    int MaxRobots);

public static class SessionResourceNames
{
    public static string Container(Guid sessionId) => $"robotswarm-{sessionId:N}";
    public static string Network(Guid sessionId) => $"robotswarm-{sessionId:N}-net";
    public static string Hostname(Guid sessionId) => $"swarm-{sessionId:N}"[..18];
}

public static class DockerArgumentBuilder
{
    public static IReadOnlyList<string> BuildCreateContainer(
        SessionContainerSpec specification,
        WorkerOptions options)
    {
        var arguments = new List<string>
        {
            "create",
            "--name", SessionResourceNames.Container(specification.SessionId),
            "--hostname", SessionResourceNames.Hostname(specification.SessionId),
            "--label", $"{SessionLabels.Managed}=true",
            "--label", $"{SessionLabels.WorkerId}={specification.WorkerId:D}",
            "--label", $"{SessionLabels.SessionId}={specification.SessionId:D}",
            "--label", $"{SessionLabels.ArenaVersion}={specification.ArenaVersion}",
            "--label", $"{SessionLabels.ImageVersion}={specification.ImageVersion ?? string.Empty}",
            "--label", $"{SessionLabels.MaxRobots}={specification.MaxRobots}",
            "--network", SessionResourceNames.Network(specification.SessionId),
            "--restart", "unless-stopped",
            "--stop-timeout", "30",
            "--init",
            "--read-only",
            "--user", options.ContainerUser,
            "--cap-drop", "ALL",
            "--security-opt", "no-new-privileges:true",
            "--cpus", options.ContainerCpuLimit.ToString("0.###", CultureInfo.InvariantCulture),
            "--memory", options.ContainerMemory,
            "--memory-swap", options.ContainerMemorySwap,
            "--pids-limit", options.ContainerPidsLimit.ToString(CultureInfo.InvariantCulture),
            "--shm-size", options.ContainerShmSize,
            "--ulimit", "nofile=4096:4096",
            "--log-driver", "local",
            "--log-opt", "max-size=20m",
            "--log-opt", "max-file=5",
            "--tmpfs", "/tmp:rw,nosuid,nodev,noexec,size=1g,mode=1777",
            "--tmpfs", "/run:rw,nosuid,nodev,noexec,size=64m,mode=0755",
            "--env", "HOME=/tmp",
            "--env", "ROS_HOME=/tmp/ros",
            "--env", "USER=swarm",
            "--env", "LOGNAME=swarm",
            "--env", "TURTLEBOT3_MODEL=burger",
            "--env", "ROS_MASTER_URI=http://127.0.0.1:11311",
            "--env", "GAZEBO_MODEL_DATABASE_URI="
        };

        if (options.EnableGpu)
        {
            arguments.Add("--gpus");
            arguments.Add(options.GpuRequest);
            arguments.Add("--env");
            arguments.Add("NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,video");
        }

        arguments.Add(specification.Image);
        arguments.Add("roslaunch");
        arguments.Add("robot_swarm_bridge");
        arguments.Add("swarm_main.launch");
        arguments.Add("robot_count:=0");
        arguments.Add($"max_robots:={specification.MaxRobots}");
        arguments.Add("robot_model:=burger");
        arguments.Add("gui:=false");
        arguments.Add("rviz:=false");
        arguments.Add("start_legacy_bridge:=false");
        arguments.Add("backend_url:=ws://127.0.0.1:9/hubs/robot");

        return arguments;
    }

    public static IReadOnlyList<string> BuildCreateNetwork(
        Guid workerId,
        Guid sessionId)
    {
        return new[]
        {
            "network",
            "create",
            "--driver",
            "bridge",
            "--internal",
            "--label",
            $"{SessionLabels.Managed}=true",
            "--label",
            $"{SessionLabels.WorkerId}={workerId:D}",
            "--label",
            $"{SessionLabels.SessionId}={sessionId:D}",
            SessionResourceNames.Network(sessionId)
        };
    }
}
