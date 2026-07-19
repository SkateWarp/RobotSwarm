using Microsoft.EntityFrameworkCore;
using Microsoft.AspNetCore.HttpOverrides;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Routes;
using SwarmBackend.Services;
using System.Text.Json;
using System.Net;

var builder = WebApplication.CreateBuilder(args);

var corsOrigins = builder.Configuration
    .GetSection("Cors:Origins")
    .GetChildren()
    .Select(origin => origin.Value?.TrimEnd('/'))
    .Where(origin => !string.IsNullOrWhiteSpace(origin))
    .Cast<string>()
    .Distinct(StringComparer.OrdinalIgnoreCase)
    .ToArray();
if (corsOrigins.Length == 0 && builder.Environment.IsDevelopment())
{
    corsOrigins =
    [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:5173",
        "http://127.0.0.1:5173"
    ];
}
else if (corsOrigins.Length == 0)
{
    throw new InvalidOperationException(
        "Cors:Origins must contain at least one frontend origin outside Development.");
}

builder.Services
    .AddDbContext<DataContext>(options => { options.UseNpgsql(builder.Configuration.GetConnectionString("Default")); });

AppContext.SetSwitch("Npgsql.EnableLegacyTimestampBehavior", true);

builder.Services.AddHttpContextAccessor();
builder.Services.AddAuthorization();
builder.Services.AddAbuseProtection(builder.Configuration);
builder.Services.GetConfigureJwt(builder.Configuration);
builder.Services.AddControllers();
builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<ISensorService, SensorService>();
builder.Services.AddScoped<ISensorReadingService, SensorReadingService>();
builder.Services.AddScoped<ITaskLogService, TaskLogService>();
builder.Services.AddScoped<ITaskTemplateService, TaskTemplateService>();
builder.Services.AddScoped<IRealtimeService, RobotHub>();
builder.Services.AddScoped<IRobotGroupService, RobotGroupService>();
builder.Services.AddScoped<WorkerCommandService>();
builder.Services.AddSingleton<ViewerControlRegistry>();
builder.Services.AddViewerHlsProxy();
builder.Services.AddHostedService<SimulationSessionScheduler>();
builder.Services.AddHostedService<TaskOutcomeMonitor>();
builder.Services.AddHostedService<ViewerControlReconciler>();
builder.Services.AddSignalR(hubOptions =>
{
    hubOptions.EnableDetailedErrors = builder.Environment.IsDevelopment();
    hubOptions.KeepAliveInterval = TimeSpan.FromSeconds(15);
    hubOptions.HandshakeTimeout = TimeSpan.FromSeconds(15);
    hubOptions.MaximumParallelInvocationsPerClient = 1;
})
.AddJsonProtocol(options =>
{
    options.PayloadSerializerOptions.PropertyNamingPolicy = JsonNamingPolicy.CamelCase;
    options.PayloadSerializerOptions.PropertyNameCaseInsensitive = true;
});
builder.Services.AddCors(options =>
{
    options.AddPolicy("Frontend", policy =>
    {
        policy.WithOrigins(corsOrigins)
            .AllowAnyMethod()
            .AllowAnyHeader()
            .AllowCredentials();
    });
});
builder.Services.AddHealthChecks();

var knownProxyAddresses = builder.Configuration
    .GetSection("ReverseProxy:KnownProxies")
    .GetChildren()
    .Select(proxy => proxy.Value)
    .Where(proxy => !string.IsNullOrWhiteSpace(proxy))
    .Select(proxy => IPAddress.TryParse(proxy, out var address)
        ? address
        : throw new InvalidOperationException(
            $"ReverseProxy:KnownProxies contains an invalid IP address: '{proxy}'."))
    .ToArray();
builder.Services.Configure<ForwardedHeadersOptions>(options =>
{
    options.ForwardedHeaders =
        ForwardedHeaders.XForwardedFor | ForwardedHeaders.XForwardedProto;
    options.ForwardLimit = 1;
    foreach (var address in knownProxyAddresses)
    {
        if (!options.KnownProxies.Contains(address))
        {
            options.KnownProxies.Add(address);
        }
    }
});

if (builder.Environment.IsDevelopment())
{
    builder.Services.AddEndpointsApiExplorer();
    builder.Services.ConfigureSwagger();
}

var app = builder.Build();
var legacyControlEnabled = app.Configuration.GetValue<bool>("LegacyControl:Enabled");
if (legacyControlEnabled && !app.Environment.IsDevelopment())
{
    app.Logger.LogWarning(
        "Legacy ROS control is enabled for rollout compatibility. Disable it after the session workspace is live.");
}

await using (var scope = app.Services.CreateAsyncScope())
{
    var dataContext = scope.ServiceProvider.GetRequiredService<DataContext>();
    await dataContext.Database.MigrateAsync();
    await AdminBootstrapper.EnsureAdminExists(
        dataContext,
        app.Configuration,
        app.Logger);
}

app.UseForwardedHeaders();
app.UseRouting();
app.UseCors("Frontend");
app.UseWebSockets(new WebSocketOptions
{
    KeepAliveInterval = TimeSpan.FromSeconds(120),
});
app.UseAuthentication();
app.UseRateLimiter();
app.UseAuthorization();

if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

app.MapGroup("Accounts")
    .MapAccount();
app.MapGroup("Robots")
    .MapRobot();
app.MapGroup("Sensors")
    .MapSensor();
app.MapGroup("SensorReadings")
    .MapSensorReading();
app.MapGroup("TaskTemplate")
    .MapTaskTemplate();
app.MapGroup("TaskLog")
    .MapTaskLog();
app.MapGroup("RobotGroups")
    .MapRobotGroup();
app.MapGroup("/api/sessions")
    .MapSimulationSession()
    .MapSessionControl();
app.MapGroup("/api/workers")
    .MapComputeWorker();
app.MapGroup("/api/worker-maintenance")
    .MapWorkerMaintenance();
app.MapGroup("/api/viewer")
    .MapViewerAuth()
    .MapViewerHlsProxy();

app.MapHub<SessionHub>("/hubs/session");
app.MapHub<WorkerHub>("/hubs/worker");
if (legacyControlEnabled)
{
    app.MapGroup("WebSocket")
        .MapWebSocket();
    app.MapHub<RobotHub>("/hubs/robot");
    app.MapGet("/hubs/robot/test", () => "Legacy SignalR Hub is running");
}

app.MapHealthChecks("/health");
app.MapControllers();

app.Run();
