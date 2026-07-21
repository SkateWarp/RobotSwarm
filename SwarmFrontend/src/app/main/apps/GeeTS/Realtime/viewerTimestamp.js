const DOTNET_TIMESTAMP_WITHOUT_ZONE = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?$/i;

export const normalizeViewerTimestamp = (value) => {
    if (typeof value !== "string") return value;

    const timestamp = value.trim();
    if (DOTNET_TIMESTAMP_WITHOUT_ZONE.test(timestamp)) {
        return `${timestamp}Z`;
    }

    return timestamp;
};

export const parseViewerTimestamp = (value) => Date.parse(normalizeViewerTimestamp(value));
