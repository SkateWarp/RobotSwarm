using System.Text.Json;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class WorkerCommandService(DataContext dataContext)
{
    public Task<WorkerCommand?> Find(
        Guid sessionId,
        string idempotencyKey,
        CancellationToken cancellationToken)
    {
        return dataContext.WorkerCommands
            .Include(command => command.TaskRun)
            .SingleOrDefaultAsync(
                command => command.SimulationSessionId == sessionId
                    && command.IdempotencyKey == idempotencyKey,
                cancellationToken);
    }

    public async Task<(WorkerCommand Command, bool Created)> Queue(
        SimulationSession session,
        WorkerCommandType type,
        string idempotencyKey,
        JsonDocument payload,
        TaskRun? taskRun,
        CancellationToken cancellationToken)
    {
        var existing = await dataContext.WorkerCommands
            .SingleOrDefaultAsync(
                command => command.SimulationSessionId == session.Id
                    && command.IdempotencyKey == idempotencyKey,
                cancellationToken);

        if (existing != null)
        {
            var sameRequest = existing.Type == type
                && existing.TaskRunId == taskRun?.Id
                && SamePayload(existing.Payload, payload);
            if (!sameRequest)
            {
                throw new InvalidOperationException(
                    "The Idempotency-Key was already used for a different command.");
            }

            return (existing, false);
        }

        var sequence = (await dataContext.WorkerCommands
            .Where(command => command.SimulationSessionId == session.Id)
            .Select(command => (long?)command.Sequence)
            .MaxAsync(cancellationToken) ?? 0) + 1;

        var now = DateTime.UtcNow;
        var command = new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            TaskRunId = taskRun?.Id,
            TaskRun = taskRun,
            ComputeWorkerId = session.ComputeWorkerId,
            Type = type,
            State = WorkerCommandState.Pending,
            IdempotencyKey = idempotencyKey,
            Sequence = sequence,
            Payload = payload,
            CreatedAt = now,
            UpdatedAt = now
        };
        dataContext.WorkerCommands.Add(command);
        return (command, true);
    }

    public static WorkerCommandResponse ToResponse(WorkerCommand command)
    {
        return new WorkerCommandResponse(
            command.Id,
            command.SimulationSessionId,
            command.TaskRunId,
            command.Type.ToString(),
            command.State.ToString(),
            command.IdempotencyKey,
            command.CorrelationId,
            command.Sequence,
            command.CreatedAt);
    }

    public static bool SamePayload(JsonDocument left, JsonDocument right)
    {
        return SamePayload(left.RootElement, right.RootElement);
    }

    public static bool SamePayload(JsonElement left, JsonElement right)
    {
        if (left.ValueKind != right.ValueKind)
        {
            return false;
        }

        return left.ValueKind switch
        {
            JsonValueKind.Object => SameObject(left, right),
            JsonValueKind.Array => left.EnumerateArray()
                .SequenceEqual(right.EnumerateArray(), JsonElementComparer.Instance),
            JsonValueKind.String => left.GetString() == right.GetString(),
            JsonValueKind.Number => SameNumber(left, right),
            JsonValueKind.True or JsonValueKind.False =>
                left.GetBoolean() == right.GetBoolean(),
            JsonValueKind.Null => true,
            _ => left.GetRawText() == right.GetRawText()
        };
    }

    private static bool SameObject(JsonElement left, JsonElement right)
    {
        var leftProperties = left.EnumerateObject().ToArray();
        var rightProperties = right.EnumerateObject().ToArray();
        if (leftProperties.Length != rightProperties.Length)
        {
            return false;
        }

        var rightByName = new Dictionary<string, JsonElement>(StringComparer.Ordinal);
        foreach (var property in rightProperties)
        {
            if (!rightByName.TryAdd(property.Name, property.Value))
            {
                return left.GetRawText() == right.GetRawText();
            }
        }

        foreach (var property in leftProperties)
        {
            if (!rightByName.TryGetValue(property.Name, out var rightValue)
                || !SamePayload(property.Value, rightValue))
            {
                return false;
            }
        }

        return leftProperties
            .Select(property => property.Name)
            .Distinct(StringComparer.Ordinal)
            .Count() == leftProperties.Length;
    }

    private static bool SameNumber(JsonElement left, JsonElement right)
    {
        if (left.TryGetDecimal(out var leftDecimal)
            && right.TryGetDecimal(out var rightDecimal))
        {
            return leftDecimal == rightDecimal;
        }

        return left.GetRawText() == right.GetRawText();
    }

    private sealed class JsonElementComparer : IEqualityComparer<JsonElement>
    {
        public static JsonElementComparer Instance { get; } = new();

        public bool Equals(JsonElement left, JsonElement right)
        {
            return SamePayload(left, right);
        }

        public int GetHashCode(JsonElement element)
        {
            return 0;
        }
    }
}
