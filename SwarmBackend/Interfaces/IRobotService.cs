using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IRobotService
{
    Task<IEnumerable<RobotResponse>> GetAll(int? accountId = null);
    Task<Result<RobotResponse>> GetById(int id, int? accountId = null);

    Task<RobotResponse> Create(RobotRequest request);
    Task<Result<RobotResponse>> Update(int id, RobotRequest request, int? accountId = null);

    Task<Result<RobotResponse>> Cancel(int id);




}
