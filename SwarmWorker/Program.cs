using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;
using SwarmWorker.Services;

var builder = Host.CreateApplicationBuilder(args);

builder.Services
    .AddOptions<WorkerOptions>()
    .Bind(builder.Configuration.GetSection(WorkerOptions.SectionName))
    .ValidateOnStart();
builder.Services.AddSingleton<IValidateOptions<WorkerOptions>, WorkerOptionsValidator>();

builder.Services.AddSingleton<IDockerCli, DockerCli>();
builder.Services.AddSingleton<DockerSessionManager>();
builder.Services.AddSingleton<TaskStatusTracker>();
builder.Services.AddSingleton<WorkerHubConnection>();
builder.Services.AddSingleton<SessionCommandHandler>();

builder.Services.AddSingleton<BoundedCommandExecutor>();
builder.Services.AddSingleton<IHostedService>(
    services => services.GetRequiredService<BoundedCommandExecutor>());
builder.Services.AddHostedService<WorkerAgent>();
builder.Services.AddHostedService<BackendDisconnectSafetyMonitor>();
builder.Services.AddHostedService<SessionControlHeartbeat>();
builder.Services.AddHostedService<TaskStatusPoller>();

await builder.Build().RunAsync();
