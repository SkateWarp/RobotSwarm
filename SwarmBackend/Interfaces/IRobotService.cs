using LanguageExt.Common;
using SwarmBackend.Entities;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IRobotService
{
    Task<IEnumerable<RobotResponse>> GetAll(
        int? accountId = null,
        bool? isPublic = null,
        Role? role = null);
    Task<Result<RobotResponse>> GetById(int id, int? accountId = null, Role? role = null);

    Task<Result<RobotResponse>> Create(RobotRequest request, int accountId);
    Task<Result<RobotResponse>> Update(
        int id,
        RobotRequest request,
        int accountId,
        Role role);

    Task<Result<RobotResponse>> Cancel(int id, int accountId, Role role);




}
