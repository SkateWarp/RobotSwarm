using System.Text.Json;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Diagnostics;
using Microsoft.EntityFrameworkCore.Storage.ValueConversion;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Tests;

internal sealed class TestDataContext : DataContext
{
    private TestDataContext(DbContextOptions<DataContext> options)
        : base(options)
    {
    }

    public static TestDataContext Create()
    {
        var options = new DbContextOptionsBuilder<DataContext>()
            .UseInMemoryDatabase(Guid.NewGuid().ToString("N"))
            .ConfigureWarnings(warnings => warnings.Ignore(
                InMemoryEventId.TransactionIgnoredWarning))
            .Options;
        return new TestDataContext(options);
    }

    protected override void OnModelCreating(ModelBuilder modelBuilder)
    {
        base.OnModelCreating(modelBuilder);

        var jsonConverter = new ValueConverter<JsonDocument, string>(
            document => document.RootElement.GetRawText(),
            json => JsonDocument.Parse(json, new JsonDocumentOptions()));
        var nullableJsonConverter = new ValueConverter<JsonDocument?, string?>(
            document => document == null
                ? null
                : document.RootElement.GetRawText(),
            json => json == null
                ? null
                : JsonDocument.Parse(json, new JsonDocumentOptions()));

        modelBuilder.Entity<ComputeWorker>()
            .Property(worker => worker.Capabilities)
            .HasConversion(jsonConverter);
        modelBuilder.Entity<TaskRun>()
            .Property(task => task.Parameters)
            .HasConversion(jsonConverter);
        modelBuilder.Entity<TaskRun>()
            .Property(task => task.Result)
            .HasConversion(nullableJsonConverter);
        modelBuilder.Entity<WorkerCommand>()
            .Property(command => command.Payload)
            .HasConversion(jsonConverter);
        modelBuilder.Entity<WorkerCommand>()
            .Property(command => command.Result)
            .HasConversion(nullableJsonConverter);
        modelBuilder.Entity<TaskLog>()
            .Property(log => log.Parameters)
            .HasConversion(jsonConverter);
    }
}
