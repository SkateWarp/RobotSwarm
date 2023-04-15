using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface ITaskTemplateService {
  
    Task<IEnumerable<TaskTemplateResponse>> GetAll();
    Task<Result<TaskTemplateResponse>> Update(int id, TaskTemplateRequest request);
}