using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface ISensorReadingService
{
    Task<SensorReadingResponse> Create(SensorReadingRequest request);
    Task<SensorReadingResponse> Create(int robotId, RosSensorReadingRequest request);
    Task<IEnumerable<SensorReadingResponse>> CreateBatch(int robotId, RosBatchSensorReadingRequest request);
    Task<IEnumerable<SensorReadingResponse>> CreateFromClient(int robotId, ClientSensorReadingRequest request);
    Task<Result<SensorReadingResponse>> Update(int id, SensorReadingRequest request);
    Task<IEnumerable<SensorReadingResponse>> GetAllByRobot(int robotId, DateRangeRequest dateRange);
    Task<IEnumerable<SensorReadingResponse>> GetAllBySensor(int sensorId, DateRangeRequest dateRange);

    IEnumerable<SensorReadingResponse> GetLastByRobot(int robotId);

    IEnumerable<SensorReadingResponse> GetLastByRobotAndSensor(int robotId, int sensorId);
}
