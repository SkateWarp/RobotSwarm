using Microsoft.EntityFrameworkCore;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Routes;
using SwarmBackend.Services;

var builder = WebApplication.CreateBuilder(args);

builder.Services
    .AddDbContext<DataContext>(options =>
    {
        options.UseNpgsql(builder.Configuration.GetConnectionString("Default"));
    });

AppContext.SetSwitch("Npgsql.EnableLegacyTimestampBehavior", true);

builder.Services.AddHttpContextAccessor();
builder.Services.AddAuthentication();
builder.Services.AddAuthorization();
builder.Services.GetConfigureJwt(builder.Configuration);

builder.Services.AddScoped<IAccountService, AccountService>();
builder.Services.AddScoped<IRobotService, RobotService>();
builder.Services.AddScoped<ISensorService, SensorService>();
builder.Services.AddScoped<ISensorReadingService, SensortReadingService>();





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
                 .SetIsOriginAllowed(origin => true)
                .AllowAnyMethod()
                .AllowAnyHeader()
                .AllowCredentials();
            });
    }
);


var app = builder.Build();

app.UseAuthentication();
app.UseAuthorization();
app.UseCors("AllowAll");



// Configure the HTTP request pipeline.
if (app.Environment.IsDevelopment())
{
    app.UseSwagger();
    app.UseSwaggerUI();
}

app.UseHttpsRedirection();

app.MapGroup("Accounts")
    .MapAccount();
app.MapGroup("Robots")
    .MapRobot();
app.MapGroup("Sensors")
    .MapSensor();

app.Run();
