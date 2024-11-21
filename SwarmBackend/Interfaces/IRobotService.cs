using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IRobotService
{
    Task<IEnumerable<RobotResponse>> GetAll();

    Task<RobotResponse> Create(RobotRequest request);
    Task<Result<RobotResponse>> Update(int id, RobotRequest request);

    Task<Result<RobotResponse>> Cancel(int id);




}
