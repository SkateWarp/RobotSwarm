using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Services;

internal static class TerminalCleanupPolicy
{
    internal const int MaximumAttempts = 5;
    internal static readonly TimeSpan BaseRetryDelay = TimeSpan.FromSeconds(30);
    private static readonly TimeSpan MaximumRetryDelay = TimeSpan.FromMinutes(5);

    internal static WorkerCommand? TryQueue(
        SimulationSession session,
        DateTime now,
        string idempotencyPrefix,
        bool resourceKnownPresent)
    {
        if (session.State == SimulationSessionState.Queued
            || !session.ComputeWorkerId.HasValue)
        {
            return null;
        }

        var stopCommands = session.Commands
            .Where(command => command.Type == WorkerCommandType.StopSession)
            .OrderByDescending(command => command.Sequence)
            .ToList();
        if (stopCommands.Any(command => IsInFlight(command.State))
            || stopCommands.Count >= MaximumAttempts)
        {
            return null;
        }

        var previous = stopCommands.FirstOrDefault();
        if (previous != null)
        {
            if (previous.State == WorkerCommandState.Completed
                && !resourceKnownPresent)
            {
                return null;
            }

            var retryAfter = (previous.CompletedAt ?? previous.UpdatedAt)
                + GetRetryDelay(stopCommands.Count);
            if (now < retryAfter)
            {
                return null;
            }
        }

        var attempt = stopCommands.Count + 1;
        var sequence = session.Commands.Count == 0
            ? 1
            : session.Commands.Max(command => command.Sequence) + 1;
        var command = new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            ComputeWorkerId = session.ComputeWorkerId,
            Type = WorkerCommandType.StopSession,
            State = WorkerCommandState.Pending,
            IdempotencyKey = $"{idempotencyPrefix}:{session.Id:N}:{attempt}",
            Sequence = sequence,
            Payload = JsonDocument.Parse("{}"),
            CreatedAt = now,
            UpdatedAt = now
        };
        session.Commands.Add(command);
        return command;
    }

    internal static bool NeedsAttempt(
        SimulationSession session,
        bool resourceKnownPresent)
    {
        if (resourceKnownPresent)
        {
            return true;
        }

        var latestStop = session.Commands
            .Where(command => command.Type == WorkerCommandType.StopSession)
            .OrderByDescending(command => command.Sequence)
            .FirstOrDefault();
        return latestStop == null
            || latestStop.State is WorkerCommandState.Failed
                or WorkerCommandState.Cancelled;
    }

    internal static TimeSpan GetRetryDelay(int completedAttempts)
    {
        var exponent = Math.Clamp(completedAttempts - 1, 0, 10);
        var seconds = BaseRetryDelay.TotalSeconds * Math.Pow(2, exponent);
        return TimeSpan.FromSeconds(Math.Min(
            seconds,
            MaximumRetryDelay.TotalSeconds));
    }

    private static bool IsInFlight(WorkerCommandState state)
    {
        return state is WorkerCommandState.Pending
            or WorkerCommandState.Dispatched
            or WorkerCommandState.Acknowledged
            or WorkerCommandState.Running;
    }
}
