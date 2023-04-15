using System.ComponentModel;

namespace SwarmBackend.Entities;

public enum SensorTypeEnum
{
    [Description("Ninguno")]
    None = 0,
    [Description("Velocidad")]
    Speed = 1,

    [Description("Distncia")]
    LaserDistance = 2,
    [Description("Encoder")]
    Encoder = 3,
    [Description("Inercial")]
    Inercial= 4,
    [Description("Porcentaje de velocidad en motor")]
    MotorSpeedPercentage = 5,
}
