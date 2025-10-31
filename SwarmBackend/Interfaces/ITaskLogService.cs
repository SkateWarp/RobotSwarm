using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface ITaskLogService
{

    Task<IEnumerable<TaskLogResponse>> GetAll(DateRangeRequest dateRange);
    Task<IEnumerable<TaskLogResponse>> GetByRobot(int robotId);
    Task<Result<TaskLogResponse>> Create(TaskLogRequest request);
    Task<Result<TaskLogResponse>> Create(int robotId, RosTaskTemplateRequest request);
    Task<Result<TaskLogResponse>> Update(int id, TaskLogRequest request);
    Task<Result<TaskLogResponse>> Cancel(int id);

    Task<Result<TaskLogResponse>> FinishTask(int robotId);

    Task<Result<IEnumerable<TaskLogResponse>>> CancelTasksByRobot(int robotId, int accountId);


}
