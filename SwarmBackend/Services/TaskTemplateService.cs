using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class TaskTemplateService : ITaskTemplateService{
    private readonly DataContext context;

    public TaskTemplateService(DataContext context)
    {
        this.context = context;
    }

    public async Task<IEnumerable<TaskTemplateResponse>> GetAll()
    {
        return await context.TaskTemplates
            .Select(x => TaskTemplateResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<TaskTemplateResponse>> Update(int id, TaskTemplateRequest request)
    {
        var template = await context.TaskTemplates.FindAsync(id);
        if (template == null)
        {
            return new Result<TaskTemplateResponse>(new Exception("Tarea no encontrada"));
        }

        context.Entry(template).CurrentValues.SetValues(request);
        await context.SaveChangesAsync();

        return TaskTemplateResponse.From(template);
    }
}