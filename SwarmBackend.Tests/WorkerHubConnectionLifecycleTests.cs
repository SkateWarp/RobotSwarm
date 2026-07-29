using System.Security.Claims;
using Microsoft.AspNetCore.Http.Features;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class WorkerHubConnectionLifecycleTests
{
    [Fact]
    public async Task RotationCompletedBeforeClaimCannotBeMissedByConnectionSetup()
    {
        var authenticatedAt = DateTime.UtcNow.AddMinutes(-1);
        var worker = ActiveWorker(authenticatedAt.AddSeconds(1));
        await using var dataContext = TestDataContext.Create();
        dataContext.ComputeWorkers.Add(worker);
        await dataContext.SaveChangesAsync();
        dataContext.ChangeTracker.Clear();

        var caller = new WorkerCallerContext(worker.Id, authenticatedAt);
        var groups = new RecordingGroupManager();
        var registry = Registry();
        var hub = Hub(dataContext, registry, caller, groups);

        var exception = await Assert.ThrowsAsync<HubException>(
            () => hub.OnConnectedAsync());

        Assert.Contains(
            "credential is no longer active",
            exception.Message,
            StringComparison.Ordinal);
        Assert.True(caller.IsAborted);
        Assert.Equal(0, groups.SuccessfulAdds);
        Assert.False(registry.IsCurrent(worker.Id, caller.ConnectionId));
    }

    [Fact]
    public async Task RevocationCompletedBeforeClaimCannotBeMissedByConnectionSetup()
    {
        var authenticatedAt = DateTime.UtcNow.AddMinutes(-1);
        var worker = ActiveWorker(authenticatedAt);
        worker.CredentialRevokedAt = DateTime.UtcNow;
        await using var dataContext = TestDataContext.Create();
        dataContext.ComputeWorkers.Add(worker);
        await dataContext.SaveChangesAsync();
        dataContext.ChangeTracker.Clear();

        var caller = new WorkerCallerContext(worker.Id, authenticatedAt);
        var groups = new RecordingGroupManager();
        var registry = Registry();
        var hub = Hub(dataContext, registry, caller, groups);

        await Assert.ThrowsAsync<HubException>(() => hub.OnConnectedAsync());

        Assert.True(caller.IsAborted);
        Assert.Equal(0, groups.SuccessfulAdds);
        Assert.False(registry.IsCurrent(worker.Id, caller.ConnectionId));
    }

    [Fact]
    public async Task ConcurrentInvalidationPreventsWorkerGroupMembership()
    {
        var authenticatedAt = DateTime.UtcNow.AddMinutes(-1);
        var worker = ActiveWorker(authenticatedAt);
        await using var dataContext = TestDataContext.Create();
        dataContext.ComputeWorkers.Add(worker);
        await dataContext.SaveChangesAsync();
        dataContext.ChangeTracker.Clear();

        var caller = new WorkerCallerContext(worker.Id, authenticatedAt);
        var groups = new RecordingGroupManager(blockBeforeAdd: true);
        var registry = Registry();
        var hub = Hub(dataContext, registry, caller, groups);

        var connecting = hub.OnConnectedAsync();
        await groups.AddStarted.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.True(registry.IsCurrent(worker.Id, caller.ConnectionId));

        Assert.True(registry.Invalidate(worker.Id));
        groups.AllowAdd.TrySetResult();

        await Assert.ThrowsAnyAsync<OperationCanceledException>(
            () => connecting);
        Assert.True(caller.IsAborted);
        Assert.Equal(0, groups.SuccessfulAdds);
        Assert.False(registry.IsCurrent(worker.Id, caller.ConnectionId));
    }

    private static ComputeWorker ActiveWorker(DateTime credentialCreatedAt) =>
        new()
        {
            Name = $"worker-{Guid.NewGuid():N}",
            CredentialHash = "active-credential-hash",
            CredentialCreatedAt = credentialCreatedAt
        };

    private static WorkerHub Hub(
        DataContext dataContext,
        WorkerConnectionRegistry registry,
        WorkerCallerContext caller,
        IGroupManager groups) =>
        new(
            dataContext,
            NullSessionHubContext.Instance,
            registry,
            new ConfigurationBuilder().Build(),
            NullLogger<WorkerHub>.Instance)
        {
            Context = caller,
            Groups = groups
        };

    private static WorkerConnectionRegistry Registry() =>
        new(NullLogger<WorkerConnectionRegistry>.Instance);

    private sealed class WorkerCallerContext : HubCallerContext
    {
        private readonly CancellationTokenSource _aborted = new();

        public WorkerCallerContext(Guid workerId, DateTime credentialCreatedAt)
        {
            ConnectionId = $"worker-lifecycle-{Guid.NewGuid():N}";
            User = new ClaimsPrincipal(new ClaimsIdentity(new[]
            {
                new Claim("worker_id", workerId.ToString()),
                new Claim(
                    "worker_credential_version",
                    credentialCreatedAt.Ticks.ToString()),
                new Claim(
                    WorkerCredentialDefaults.AgentInstanceClaim,
                    Guid.NewGuid().ToString("D"))
            }, "worker-lifecycle-test"));
        }

        public bool IsAborted => _aborted.IsCancellationRequested;
        public override string ConnectionId { get; }
        public override string? UserIdentifier =>
            User?.FindFirst("worker_id")?.Value;
        public override ClaimsPrincipal? User { get; }
        public override IDictionary<object, object?> Items { get; } =
            new Dictionary<object, object?>();
        public override IFeatureCollection Features { get; } =
            new FeatureCollection();
        public override CancellationToken ConnectionAborted => _aborted.Token;

        public override void Abort()
        {
            _aborted.Cancel();
        }
    }

    private sealed class RecordingGroupManager(
        bool blockBeforeAdd = false) : IGroupManager
    {
        public TaskCompletionSource AddStarted { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource AllowAdd { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        public int SuccessfulAdds { get; private set; }

        public async Task AddToGroupAsync(
            string connectionId,
            string groupName,
            CancellationToken cancellationToken = default)
        {
            AddStarted.TrySetResult();
            if (blockBeforeAdd)
            {
                await AllowAdd.Task;
            }

            cancellationToken.ThrowIfCancellationRequested();
            SuccessfulAdds++;
        }

        public Task RemoveFromGroupAsync(
            string connectionId,
            string groupName,
            CancellationToken cancellationToken = default)
        {
            return Task.CompletedTask;
        }
    }

    private sealed class NullSessionHubContext : IHubContext<SessionHub>
    {
        public static NullSessionHubContext Instance { get; } = new();
        public IHubClients Clients => null!;
        public IGroupManager Groups => null!;
    }
}
