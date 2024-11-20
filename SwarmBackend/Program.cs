using Microsoft.EntityFrameworkCore;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Routes;
using SwarmBackend.Services;

// trigger
var builder = WebApplication.CreateBuilder(args);
builder.Services.AddLogging(logging =>
{
    logging.ClearProviders();
    logging.AddConsole();
    logging.AddDebug();
    logging.SetMinimumLevel(LogLevel.Debug);
});
builder.Services
    .AddDbContext<DataContext>(options => { options.UseNpgsql(builder.Configuration.GetConnectionString("Default")); });

AppContext.SetSwitch("Npgsql.EnableLegacyTimestampBehavior", true);

builder.Services.AddHttpContextAccessor();
builder.Services.AddAuthentication();
builder.Services.AddAuthorization();
builder.Services.GetConfigureJwt(builder.Configuration);
builder.Services.AddControllers();
builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<ISensorService, SensorService>();
builder.Services.AddScoped<ISensorReadingService, SensorReadingService>();
builder.Services.AddScoped<ITaskLogService, TaskLogService>();
builder.Services.AddScoped<ITaskTemplateService, TaskTemplateService>();
builder.Services.AddScoped<IRealtimeService, RobotHub>();
builder.Services.AddSignalR()
    .AddJsonProtocol(options => { options.PayloadSerializerOptions.PropertyNamingPolicy = null; });


// Add services to the container.
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen();
builder.Services.ConfigureSwagger();
builder.Services.AddCors(
    options =>
    {
        options.AddPolicy("AllowAll",
            policyBuilder =>
            {
                policyBuilder
                    //.AllowAnyOrigin()
                    .SetIsOriginAllowed(_ => true)
                    .AllowAnyMethod()
                    .AllowAnyHeader()
                    .AllowCredentials();
            });
    }
);


var app = builder.Build();
app.UseCors("AllowAll");
app.UseRouting();
app.UseAuthentication();
app.UseAuthorization();

using (var scope = app.Services.CreateScope())
{
    var scopedContext = scope.ServiceProvider.GetRequiredService<DataContext>();
    var accountService = scope.ServiceProvider.GetRequiredService<IAccountService>();


    if (!accountService.GetAll().GetAwaiter().GetResult().Any())
    {
        await accountService.Create(Seed.GetAccount());
    }


    scopedContext.SaveChanges();
}





// Configure the HTTP request pipeline.
//if (app.Environment.IsDevelopment())
//{
app.UseSwagger();
app.UseSwaggerUI();
//}

app.UseHttpsRedirection();

app.MapGroup("Accounts")
    .MapAccount();
app.MapGroup("Robots")
    .MapRobot()
    .MapGet("/hubs/robot/test", () => "SignalR Hub is running");
app.MapGroup("Sensors")
    .MapSensor();
app.MapGroup("TaskTemplate")
    .MapTaskTemplate();
app.MapGroup("TaskLog")
    .MapTaskLog();

// Add UseEndpoints
app.MapHub<RobotHub>("/hubs/robot");
app.MapGet("/hubs/robot/test", () => "SignalR Hub is running");
app.MapControllers();

app.Run();