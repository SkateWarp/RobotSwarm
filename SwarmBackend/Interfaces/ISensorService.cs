using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface ISensorService
{

    Task<SensorResponse> Create(SensorRequest request);
    Task<Result<SensorResponse>> Update(int id, SensorRequest request);
    Task<IEnumerable<SensorResponse>> GetAllByRobot(int robotId);
}
