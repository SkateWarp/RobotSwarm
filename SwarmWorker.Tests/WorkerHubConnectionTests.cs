using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Services;

namespace SwarmWorker.Tests;

public sealed class WorkerHubConnectionTests
{
    [Fact]
    public async Task DisposeCanBeCalledMoreThanOnce()
    {
        var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            NullLogger<WorkerHubConnection>.Instance);

        await connection.DisposeAsync();
        await connection.DisposeAsync();
    }
}
