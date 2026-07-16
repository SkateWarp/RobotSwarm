using System.Text.Json;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed record CommandExecutionResult(
    JsonElement Result,
    string? CompletedSessionState,
    TaskEventReport? ImmediateTaskEvent);

public sealed class SessionCommandHandler
{
    private readonly DockerSessionManager _sessions;
    private readonly WorkerOptions _options;

    public SessionCommandHandler(
        DockerSessionManager sessions,
        IOptions<WorkerOptions> options)
    {
        _sessions = sessions;
        _options = options.Value;
    }

    public async Task<CommandExecutionResult> ExecuteAsync(
        WorkerCommandEnvelope command,
        CancellationToken cancellationToken)
    {
        switch (command.Type)
        {
            case "ProvisionSession":
            {
                var payload = CommandPayloadParser.ParseFleet(
                    command.Payload,
                    _options,
                    requireArena: true);
                var result = await _sessions.ProvisionAsync(
                    command.SessionId,
                    payload,
                    cancellationToken);
                return new CommandExecutionResult(
                    JsonSerializer.SerializeToElement(result),
                    "Ready",
                    null);
            }
            case "UpdateFleet":
            {
                var payload = CommandPayloadParser.ParseFleet(
                    command.Payload,
                    _options,
                    requireArena: false);
                var result = await _sessions.UpdateFleetAsync(
                    command.SessionId,
                    payload,
                    cancellationToken);
                return new CommandExecutionResult(
                    JsonSerializer.SerializeToElement(result),
                    null,
                    null);
            }
            case "StopSession":
            {
                var result = await _sessions.StopAsync(
                    command.SessionId,
                    cancellationToken);
                return new CommandExecutionResult(
                    JsonSerializer.SerializeToElement(result),
                    "Stopped",
                    null);
            }
            case "StartTask":
            case "PauseTask":
            case "ResumeTask":
            case "CancelTask":
            case "EmergencyStop":
            case "ResetEmergencyStop":
            {
                var rosCommand = TaskCommandParser.Parse(
                    command.Type,
                    command.Payload);
                await _sessions.PublishSwarmCommandAsync(
                    command.SessionId,
                    rosCommand.Envelope,
                    cancellationToken);

                bool? emergencyStopState = command.Type switch
                {
                    "EmergencyStop" => true,
                    "ResetEmergencyStop" => false,
                    _ => null
                };
                if (emergencyStopState.HasValue)
                {
                    await _sessions.WaitForEmergencyStopAsync(
                        command.SessionId,
                        emergencyStopState.Value,
                        cancellationToken);
                }

                var taskEvent = rosCommand.TaskRunId.HasValue
                                && rosCommand.ImmediateState is not null
                    ? new TaskEventReport(
                        command.SessionId,
                        rosCommand.TaskRunId.Value,
                        rosCommand.ImmediateState,
                        rosCommand.ImmediateProgress,
                        null,
                        null)
                    : null;
                return new CommandExecutionResult(
                    JsonSerializer.SerializeToElement(new
                    {
                        taskRunId = rosCommand.TaskRunId,
                        rosCommand = rosCommand.Command,
                        emergencyStop = emergencyStopState
                    }),
                    null,
                    taskEvent);
            }
            default:
                throw new NotSupportedException(
                    $"Worker command type '{command.Type}' is not implemented by this worker build.");
        }
    }

    public Task CleanupSessionAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        return _sessions.StopAsync(sessionId, cancellationToken);
    }
}
