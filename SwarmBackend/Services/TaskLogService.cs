using System.Text.Json;
using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class TaskLogService : ITaskLogService
{
    private readonly DataContext context;

    public TaskLogService(DataContext context)
    {
        this.context = context;
    }

    public async Task<TaskLogResponse> Create(TaskLogRequest request)
    {
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
}
