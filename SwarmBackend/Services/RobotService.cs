using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotService : IRobotService
{
    private const int MaximumNameLength = 100;
    private readonly DataContext context;
    private readonly IRealtimeService realtimeService;

    public RobotService(DataContext context, IRealtimeService realtimeService)
    {
        this.context = context;
        this.realtimeService = realtimeService;
    }

    public async Task<Result<RobotResponse>> Cancel(int id, int accountId, Role role)
    {
        var robot = await context.Robots.FirstOrDefaultAsync(x => x.Id == id);
        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (!CanManage(robot, accountId, role))
        {
            return new Result<RobotResponse>(new Exception("No tienes permiso para modificar este robot"));
        }

        if (robot.Status != RobotStatus.Working)
        {
            return new Result<RobotResponse>(RobotResponse.From(robot));
        }

        robot.Status = RobotStatus.Idle;
        context.Robots.Update(robot);
        await context.SaveChangesAsync();
        await realtimeService.NotifyRobotsAvailable();

        return RobotResponse.From(robot);
    }

    public async Task<Result<RobotResponse>> Create(RobotRequest request, int accountId)
    {
        var validation = ValidateRequest(request, validateStatus: false);
        if (validation != null)
        {
            return new Result<RobotResponse>(validation);
        }

        request = Normalize(request);
        var robot = new Robot
        {
            Name = request.Name,
            Description = request.Description,
            Notes = request.Notes,
            DateCreated = DateTime.UtcNow,
            Status = RobotStatus.Idle,
            IsPublic = request.IsPublic,
            AccountId = accountId
        };

        context.Robots.Add(robot);
        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }

    public async Task<IEnumerable<RobotResponse>> GetAll(
        int? accountId = null,
        bool? isPublic = null,
        Role? role = null)
    {
        var query = context.Robots
            .Include(x => x.Sensors)
            .Where(x => x.Status != RobotStatus.Disabled);

        if (role == Role.Admin)
        {
            if (isPublic.HasValue)
            {
                query = query.Where(x => x.IsPublic == isPublic.Value);
            }
        }
        else if (accountId.HasValue)
        {
            // If user is logged in, show their robots plus public ones (unless filtered)
            if (isPublic.HasValue)
            {
                query = isPublic.Value
                    ? query.Where(x => x.IsPublic)
                    : query.Where(x => x.AccountId == accountId);
            }
            else
            {
                query = query.Where(x => x.AccountId == accountId || x.IsPublic);
            }
        }
        else
        {
            // If no user is logged in, only show public robots
            query = query.Where(x => x.IsPublic);
        }

        return await query
            .Select(x => RobotResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<RobotResponse>> GetById(
        int id,
        int? accountId = null,
        Role? role = null)
    {
        var robot = await context.Robots
            .Include(x => x.Sensors)
            .FirstOrDefaultAsync(x => x.Id == id);

        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (role != Role.Admin && !robot.IsPublic && robot.AccountId != accountId)
        {
            return new Result<RobotResponse>(new Exception("No tienes acceso a este robot"));
        }

        return RobotResponse.From(robot);
    }

    public async Task<Result<RobotResponse>> Update(
        int id,
        RobotRequest request,
        int accountId,
        Role role)
    {
        var validation = ValidateRequest(request, validateStatus: true);
        if (validation != null)
        {
            return new Result<RobotResponse>(validation);
        }

        var robot = await context.Robots.FirstOrDefaultAsync(x => x.Id == id);
        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (!CanManage(robot, accountId, role))
        {
            return new Result<RobotResponse>(new Exception("No tienes permiso para modificar este robot"));
        }

        request = Normalize(request);

        robot.Name = request.Name;
        robot.Description = request.Description;
        robot.Notes = request.Notes;
        robot.Status = request.Status;
        robot.IsPublic = request.IsPublic;

        await context.SaveChangesAsync();
        await realtimeService.NotifyRobotsAvailable();

        return RobotResponse.From(robot);
    }

    private static bool CanManage(Robot robot, int accountId, Role role)
    {
        return role == Role.Admin || robot.AccountId == accountId;
    }

    private static Exception? ValidateRequest(RobotRequest request, bool validateStatus)
    {
        if (string.IsNullOrWhiteSpace(request.Name))
        {
            return new Exception("El nombre del robot es obligatorio");
        }

        if (request.Name.Trim().Length > MaximumNameLength)
        {
            return new Exception($"El nombre no puede superar {MaximumNameLength} caracteres");
        }

        if (validateStatus && !Enum.IsDefined(request.Status))
        {
            return new Exception("El estado del robot no es válido");
        }

        return null;
    }

    private static RobotRequest Normalize(RobotRequest request)
    {
        return request with
        {
            Name = request.Name.Trim(),
            Description = Clean(request.Description),
            Notes = Clean(request.Notes)
        };
    }

    private static string? Clean(string? value)
    {
        return string.IsNullOrWhiteSpace(value) ? null : value.Trim();
    }
}
