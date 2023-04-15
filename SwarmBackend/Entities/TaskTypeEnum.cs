using System.ComponentModel;

namespace SwarmBackend.Entities;

public enum TaskTypeEnum
{
    [Description("Ninguno")]
    None = 0,
    [Description("Transporte")]
    Transport = 1,
    [Description("Sigue al líder")]
    FollowLeader = 2, 
    [Description("Formación")]
    Formation = 3,
}
