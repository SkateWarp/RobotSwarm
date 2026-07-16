using System.Text.Json;

namespace SwarmBackend.Helpers;

public static class ControlPlaneJson
{
    public static JsonDocument EmptyObject()
    {
        return JsonDocument.Parse("{}");
    }

    public static JsonDocument ToDocument(JsonElement? element)
    {
        return element.HasValue && element.Value.ValueKind is not JsonValueKind.Null
            and not JsonValueKind.Undefined
            ? JsonDocument.Parse(element.Value.GetRawText())
            : EmptyObject();
    }

    public static JsonElement ToElement(JsonDocument document)
    {
        return document.RootElement.Clone();
    }

    public static JsonElement? ToNullableElement(JsonDocument? document)
    {
        return document?.RootElement.Clone();
    }
}
