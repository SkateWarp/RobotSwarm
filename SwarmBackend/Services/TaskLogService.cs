using System.Text.Json;
using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class TaskLogService(DataContext context) : ITaskLogService
{
    public async Task<Result<TaskLogResponse>> Create(TaskLogRequest request)
    {
        var robot = await context.Robots.FindAsync(request.RobotId);
        if (robot == null)
        {
            return new Result<TaskLogResponse>(new Exception("Robot no encontrado"));
        }

        var task = new TaskLog
        {
            RobotId = request.RobotId,
            TaskTemplateId = request.TaskTemplateId,
            DateCreated = DateTime.Now,
            Parameters = JsonDocument.Parse(request.Parameters.GetRawText())
        };

        context.TaskLogs.Add(task);
        await context.SaveChangesAsync();

        return TaskLogResponse.From(task);
    }

    public async Task<IEnumerable<TaskLogResponse>> GetAll(DateRangeRequest dateRange)
    {
        var query = context.TaskLogs
            .Include(x => x.TaskTemplate)
            .Include(x => x.Robot)
            .AsQueryable();

        if (dateRange.StartDate != null)
        {
            query = query.Where(x => x.DateCreated >= dateRange.StartDate);
        }

        if (dateRange.EndDate != null)
        {
            query = query.Where(x => x.DateCreated <= dateRange.EndDate);
        }


        return await query
            .Select(x => TaskLogResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<TaskLogResponse>> Update(int id, TaskLogRequest request)
    {
        var task = await context.TaskLogs.FindAsync(id);
        if (task == null)
        {
            return new Result<TaskLogResponse>(new Exception("Tarea no encontrada"));
        }

        context.Entry(task).CurrentValues.SetValues(request);
        await context.SaveChangesAsync();

        return TaskLogResponse.From(task);
    }

    public async Task<Result<TaskLogResponse>> Cancel(int id)
    {
        {
            var task = await context.TaskLogs.FindAsync(id);
            if (task == null)
            {
                return new Result<TaskLogResponse>(new Exception("Tarea no encontrada"));
            }

            if (task.DateFinished != null)
            {
                return new Result<TaskLogResponse>(new Exception("La tarea ya ha sido cancelada"));
            }

            context.Entry(task).CurrentValues.SetValues(
                new
                {
                    DateCancelled = DateTime.Now
                }
            );
            await context.SaveChangesAsync();

            return TaskLogResponse.From(task);
        }
    }

    public async Task<Result<TaskLogResponse>> Create(int robotId, RosTaskTemplateRequest request)
    {
        if (!TaskTypeEnumUtils.TryParseFromString(request.TaskType, out var taskType))
        {
            return new Result<TaskLogResponse>(new Exception("Task type not recognized"));
        }

        var template = await context.TaskTemplates.FirstOrDefaultAsync(x => x.TaskType == taskType);
        if (template == null)
        {
            template = new TaskTemplate
            {
                TaskType = taskType,
                Name = taskType.GetDescriptionAttribute()
            };
            context.TaskTemplates.Add(template);
            await context.SaveChangesAsync();
        }

        var task = new TaskLog
        {
            RobotId = robotId,
            TaskTemplateId = template.Id,
            DateCreated = DateTime.Now,
            Parameters = JsonDocument.Parse(request.Parameters.GetRawText())
        };

        context.TaskLogs.Add(task);
        await context.SaveChangesAsync();

        var robot = await context.Robots.FindAsync(robotId);
        if (robot == null)
        {
            return new Result<TaskLogResponse>(new Exception("Robot no encontrado"));
        }

        context.Entry(robot).CurrentValues.SetValues(
            new
            {
                IsConnected = true,
                Status = RobotStatus.Working
            }
        );


        await context.SaveChangesAsync();

        return TaskLogResponse.From(task);
    }

    public async Task<Result<TaskLogResponse>> FinishTask(int robotId)
    {
        var taskLog = await context.TaskLogs.FirstOrDefaultAsync(x => x.RobotId == robotId);
        if (taskLog == null)
        {
            return new Result<TaskLogResponse>(new Exception("Tarea no encontrada"));
        }

        context.Entry(taskLog).CurrentValues.SetValues(
            new
            {
                DateFinished = DateTime.Now,
            }
        );

        var robot = await context.Robots.FindAsync(robotId);
        if (robot == null)
        {
            return new Result<TaskLogResponse>(new Exception("Robot no encontrado"));
        }

        context.Entry(robot).CurrentValues.SetValues(
            new
            {
                IsConnected = true,
                Status = RobotStatus.Idle
            }
        );

        await context.SaveChangesAsync();

        return TaskLogResponse.From(taskLog);
    }

}
