using System.ComponentModel;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class EnumExtension
{
    public static string GetDescriptionAttribute(this Enum value)
    {
        var type = value.GetType();
        var memberInfo = type.GetMember(value.ToString());
        var attributes = memberInfo[0].GetCustomAttributes(typeof(DescriptionAttribute), false);

        return attributes.Length > 0
            ? ((DescriptionAttribute)attributes[0]).Description
            : value.ToString();
    }
}

public static class SensorTypeEnumUtils
{
    public static bool TryParseFromString(string input, out SensorTypeEnum result)
    {
        result = SensorTypeEnum.None;

        if (string.IsNullOrWhiteSpace(input))
            return false;

        // First try direct enum parse
        if (Enum.TryParse(input, true, out result))
            return true;

        // Then try to match by description
        foreach (SensorTypeEnum value in Enum.GetValues(typeof(SensorTypeEnum)))
        {
            if (!value.GetDescriptionAttribute().Equals(input, StringComparison.OrdinalIgnoreCase)) continue;

            result = value;
            return true;
        }


        return false;
    }
}