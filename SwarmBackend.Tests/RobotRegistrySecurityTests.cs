using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class RobotRegistrySecurityTests
{
    [Fact]
    public async Task UserCannotModifyOrCancelPublicRobotOwnedByAnotherAccount()
    {
        await using var dataContext = TestDataContext.Create();
        var robot = new Robot
        {
            Name = "tb3_publico",
            AccountId = 22,
            IsPublic = true,
            Status = RobotStatus.Working
        };
        dataContext.Robots.Add(robot);
        await dataContext.SaveChangesAsync();
        var realtime = new RealtimeRecorder();
        var service = new RobotService(dataContext, realtime);

        var updated = await service.Update(
            robot.Id,
            new RobotRequest("Alterado", null, null, RobotStatus.Idle, false),
            accountId: 11,
            Role.User);
        var cancelled = await service.Cancel(robot.Id, accountId: 11, Role.User);

        Assert.Contains("permiso", Failure(updated).Message);
        Assert.Contains("permiso", Failure(cancelled).Message);
        Assert.Equal("tb3_publico", robot.Name);
        Assert.Equal(RobotStatus.Working, robot.Status);
        Assert.Equal(0, realtime.AvailabilityNotifications);
    }

    [Fact]
    public async Task AdministratorCanModifyAndCancelRobotOwnedByAnotherAccount()
    {
        await using var dataContext = TestDataContext.Create();
        var robot = new Robot
        {
            Name = "tb3_0",
            AccountId = 22,
            IsPublic = false,
            Status = RobotStatus.Idle
        };
        dataContext.Robots.Add(robot);
        await dataContext.SaveChangesAsync();
        var realtime = new RealtimeRecorder();
        var service = new RobotService(dataContext, realtime);

        var updated = Success(await service.Update(
            robot.Id,
            new RobotRequest("tb3_actualizado", "Patrulla", null, RobotStatus.Working, true),
            accountId: 1,
            Role.Admin));
        var cancelled = Success(await service.Cancel(robot.Id, accountId: 1, Role.Admin));

        Assert.Equal("tb3_actualizado", updated.Name);
        Assert.True(updated.IsPublic);
        Assert.Equal(22, updated.AccountId);
        Assert.Equal(RobotStatus.Idle, cancelled.Status);
        Assert.Equal(2, realtime.AvailabilityNotifications);
    }

    [Fact]
    public async Task AdministratorListsEveryActiveRobotWhileUserOnlySeesOwnedAndPublic()
    {
        await using var dataContext = TestDataContext.Create();
        dataContext.Robots.AddRange(
            new Robot { Name = "publico", AccountId = 22, IsPublic = true },
            new Robot { Name = "propio", AccountId = 11, IsPublic = false },
            new Robot { Name = "privado-ajeno", AccountId = 22, IsPublic = false },
            new Robot
            {
                Name = "deshabilitado",
                AccountId = 22,
                IsPublic = true,
                Status = RobotStatus.Disabled
            });
        await dataContext.SaveChangesAsync();
        var service = new RobotService(dataContext, new RealtimeRecorder());

        var administrator = (await service.GetAll(
            role: Role.Admin,
            includeDisabled: true)).ToArray();
        var user = (await service.GetAll(accountId: 11, role: Role.User)).ToArray();
        var anonymous = (await service.GetAll()).ToArray();

        Assert.Equal(
            new[] { "deshabilitado", "privado-ajeno", "propio", "publico" },
            Names(administrator));
        Assert.Equal(new[] { "propio", "publico" }, Names(user));
        Assert.Equal(new[] { "publico" }, Names(anonymous));
    }

    [Fact]
    public async Task DisabledInventoryIsNeverExposedToAUserOrAnonymousCaller()
    {
        await using var dataContext = TestDataContext.Create();
        dataContext.Robots.AddRange(
            new Robot { Name = "activo", AccountId = 11, IsPublic = true },
            new Robot
            {
                Name = "deshabilitado",
                AccountId = 11,
                IsPublic = true,
                Status = RobotStatus.Disabled
            });
        await dataContext.SaveChangesAsync();
        var service = new RobotService(dataContext, new RealtimeRecorder());

        var user = await service.GetAll(
            accountId: 11,
            role: Role.User,
            includeDisabled: true);
        var anonymous = await service.GetAll(includeDisabled: true);

        Assert.Equal(new[] { "activo" }, Names(user));
        Assert.Equal(new[] { "activo" }, Names(anonymous));
    }

    [Fact]
    public async Task AdministratorCanReactivateWithoutChangingRobotIdentityOrMembership()
    {
        await using var dataContext = TestDataContext.Create();
        var group = new RobotGroup { Name = "Exploradores" };
        var robot = new Robot
        {
            Name = "tb3_5",
            Namespace = "tb3_5",
            AccountId = 22,
            RobotGroup = group,
            IsPublic = false,
            Status = RobotStatus.Disabled
        };
        dataContext.AddRange(group, robot);
        await dataContext.SaveChangesAsync();
        var realtime = new RealtimeRecorder();
        var service = new RobotService(dataContext, realtime);

        var reactivated = Success(await service.Update(
            robot.Id,
            new RobotRequest(
                robot.Name,
                robot.Description,
                robot.Notes,
                RobotStatus.Idle,
                robot.IsPublic),
            accountId: 1,
            Role.Admin));

        Assert.Equal(RobotStatus.Idle, reactivated.Status);
        Assert.Equal(22, reactivated.AccountId);
        Assert.Equal("tb3_5", reactivated.Namespace);
        Assert.Equal(group.Id, robot.RobotGroupId);
        Assert.Equal(1, realtime.AvailabilityNotifications);
    }

    [Fact]
    public async Task CreateUsesAuthenticatedOwnerAndStartsRobotIdle()
    {
        await using var dataContext = TestDataContext.Create();
        var service = new RobotService(dataContext, new RealtimeRecorder());

        var created = Success(await service.Create(
            new RobotRequest(
                "tb3_2",
                null,
                null,
                RobotStatus.Working,
                IsPublic: true,
                AccountId: 999),
            accountId: 11));

        Assert.Equal(11, created.AccountId);
        Assert.Equal(RobotStatus.Idle, created.Status);
        Assert.True(created.IsPublic);
    }

    [Theory]
    [InlineData("")]
    [InlineData("   ")]
    public async Task CreateRejectsBlankNames(string name)
    {
        await using var dataContext = TestDataContext.Create();
        var service = new RobotService(dataContext, new RealtimeRecorder());

        var result = await service.Create(
            new RobotRequest(name, null, null, RobotStatus.Idle),
            accountId: 11);

        Assert.Contains("obligatorio", Failure(result).Message);
        Assert.Empty(dataContext.Robots);
    }

    [Fact]
    public async Task UpdateRejectsUnknownStatusWithoutChangingRobot()
    {
        await using var dataContext = TestDataContext.Create();
        var robot = new Robot { Name = "tb3_3", AccountId = 11 };
        dataContext.Robots.Add(robot);
        await dataContext.SaveChangesAsync();
        var service = new RobotService(dataContext, new RealtimeRecorder());

        var result = await service.Update(
            robot.Id,
            new RobotRequest("Alterado", null, null, (RobotStatus)99),
            accountId: 11,
            Role.User);

        Assert.Contains("estado", Failure(result).Message);
        Assert.Equal("tb3_3", robot.Name);
    }

    [Fact]
    public async Task RouteReturnsSerializableErrorAndPassesAuthenticatedIdentity()
    {
        var services = new ServiceCollection()
            .AddLogging()
            .AddRouting()
            .BuildServiceProvider();
        var context = new DefaultHttpContext
        {
            RequestServices = services,
            User = new ClaimsPrincipal(new ClaimsIdentity(
                new[]
                {
                    new Claim("id", "11"),
                    new Claim(ClaimTypes.Role, Role.User.ToString())
                },
                authenticationType: "test"))
        };
        context.Response.Body = new MemoryStream();
        var service = new DenyingRobotService();

        var result = await RobotRoute.Update(
            4,
            new RobotRequest("tb3_4", null, null, RobotStatus.Idle),
            service,
            context);
        await result.ExecuteAsync(context);

        context.Response.Body.Position = 0;
        using var body = await JsonDocument.ParseAsync(context.Response.Body);
        Assert.Equal(StatusCodes.Status400BadRequest, context.Response.StatusCode);
        Assert.Equal("Operación rechazada", body.RootElement.GetProperty("message").GetString());
        Assert.Equal(11, service.AccountId);
        Assert.Equal(Role.User, service.CapturedRole);
    }

    private static string[] Names(IEnumerable<RobotResponse> robots)
    {
        return robots.Select(robot => robot.Name).OrderBy(name => name).ToArray();
    }

    private static RobotResponse Success(LanguageExt.Common.Result<RobotResponse> result)
    {
        return result.Match(value => value, error => throw error);
    }

    private static Exception Failure(LanguageExt.Common.Result<RobotResponse> result)
    {
        return result.Match<Exception>(
            _ => throw new InvalidOperationException("Se esperaba un error"),
            error => error);
    }

    private sealed class RealtimeRecorder : IRealtimeService
    {
        public int AvailabilityNotifications { get; private set; }

        public Task<List<int>> NotifyRobotsAvailable()
        {
            AvailabilityNotifications++;
            return Task.FromResult(new List<int>());
        }

        public Task OnConnectedAsync() => Task.CompletedTask;
        public Task OnDisconnectedAsync(Exception? exception) => Task.CompletedTask;
        public Task UpdateRobotConnection(int robotId, bool isConnected) => Task.CompletedTask;
        public Task UpdateStatus(int robotId, string status) => Task.CompletedTask;
        public Task HandleSensorReading(int robotId, RosSensorReadingRequest reading) => Task.CompletedTask;
        public Task HandleSensorReadingFromClient(
            int robotId,
            string sensorName,
            Dictionary<string, object> sensorFields) => Task.CompletedTask;
        public Task HandleSensorReadingsBatch(int robotId, RosBatchSensorReadingRequest request) => Task.CompletedTask;
        public Task HandleTaskLog(int robotId, RosTaskTemplateRequest request) => Task.CompletedTask;
        public Task HandleFinishTaskLog(int robotId) => Task.CompletedTask;
        public Task HandleCancelTaskLog(int robotId, int accountId) => Task.CompletedTask;
        public Task SendCommand(int robotId, string command, string parameters) => Task.CompletedTask;
        public Task<bool> SpawnRobots(int count, string pattern) => Task.FromResult(true);
        public Task<bool> SpawnRobotsWithIds(
            List<RobotDeploymentInfo> robots,
            string pattern) => Task.FromResult(true);
        public Task<bool> DeleteRobots(List<string> robotIds) => Task.FromResult(true);
        public Task<bool> StartFollowLeader(
            int robotCount,
            string leaderMode,
            object config) => Task.FromResult(true);
        public Task<bool> StartFormation(
            int robotCount,
            string formationType,
            string movementMode,
            object config) => Task.FromResult(true);
        public Task<bool> StartTransport(
            int robotCount,
            double targetX,
            double targetY,
            object config) => Task.FromResult(true);
        public Task<bool> StopTask() => Task.FromResult(true);
        public Task<bool> EmergencyStop() => Task.FromResult(true);
        public Task<bool> ControlLeader(
            double linearVelocity,
            double angularVelocity) => Task.FromResult(true);
        public Task<bool> SpawnObstacles(string density) => Task.FromResult(true);
        public Task<bool> GetRosConnectionStatus() => Task.FromResult(true);
        public Task ForwardSwarmStatus(string statusJson) => Task.CompletedTask;
        public Task ForwardFleetEvent(string eventJson) => Task.CompletedTask;
    }

    private sealed class DenyingRobotService : IRobotService
    {
        public int? AccountId { get; private set; }
        public Role? CapturedRole { get; private set; }

        public Task<IEnumerable<RobotResponse>> GetAll(
            int? accountId = null,
            bool? isPublic = null,
            Role? role = null,
            bool includeDisabled = false) => throw new NotSupportedException();

        public Task<LanguageExt.Common.Result<RobotResponse>> GetById(
            int id,
            int? accountId = null,
            Role? role = null) => throw new NotSupportedException();

        public Task<LanguageExt.Common.Result<RobotResponse>> Create(
            RobotRequest request,
            int accountId) =>
            throw new NotSupportedException();

        public Task<LanguageExt.Common.Result<RobotResponse>> Update(
            int id,
            RobotRequest request,
            int accountId,
            Role role)
        {
            AccountId = accountId;
            CapturedRole = role;
            return Task.FromResult(new LanguageExt.Common.Result<RobotResponse>(
                new Exception("Operación rechazada")));
        }

        public Task<LanguageExt.Common.Result<RobotResponse>> Cancel(
            int id,
            int accountId,
            Role role) => throw new NotSupportedException();
    }
}
