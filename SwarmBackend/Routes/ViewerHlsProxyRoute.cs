using System.Net.Http.Headers;
using Microsoft.AspNetCore.Http.Extensions;
using Microsoft.Extensions.Primitives;
using SwarmBackend.Helpers;
using SwarmBackend.Services;

namespace SwarmBackend.Routes;

public static class ViewerHlsProxyRoute
{
    private static readonly string[] AllowedQueryKeys =
    [
        "_HLS_msn",
        "_HLS_part",
        "_HLS_skip"
    ];

    public static RouteGroupBuilder MapViewerHlsProxy(this RouteGroupBuilder group)
    {
        group.MapGet("/hls/{**assetPath}", Proxy)
            .RequireRateLimiting(AbuseProtection.ViewerHlsPolicy);
        return group;
    }

    internal static async Task<IResult> Proxy(
        string? assetPath,
        HttpContext context,
        DataContext dataContext,
        IConfiguration configuration,
        ViewerHlsProxy proxy,
        CancellationToken cancellationToken)
    {
        if (!configuration.GetValue<bool>("Viewer:HlsProxyEnabled"))
        {
            return NoStore(StatusCodes.Status404NotFound);
        }

        if (!TryBuildRelativePath(
                assetPath,
                context.Request.Query,
                out var streamAddress,
                out var relativePath,
                out var playlist))
        {
            return NoStore(StatusCodes.Status404NotFound);
        }

        if (!TryReadBearerToken(context.Request.Headers.Authorization, out var token)
            || !await ViewerAuthRoute.IsReadAuthorized(
                token,
                streamAddress,
                dataContext,
                cancellationToken))
        {
            return NoStore(StatusCodes.Status401Unauthorized);
        }

        if (!proxy.IsConfigured)
        {
            return NoStore(StatusCodes.Status503ServiceUnavailable);
        }

        var upstream = await proxy.Fetch(
            relativePath,
            streamAddress.SessionId,
            playlist,
            cancellationToken);
        return new ViewerHlsResult(upstream);
    }

    internal static bool TryBuildRelativePath(
        string? assetPath,
        IQueryCollection query,
        out ViewerStreamAddress streamAddress,
        out string relativePath,
        out bool playlist)
    {
        streamAddress = null!;
        relativePath = string.Empty;
        playlist = false;

        if (string.IsNullOrWhiteSpace(assetPath)
            || assetPath.Length > 360
            || assetPath != assetPath.Trim('/')
            || assetPath.Contains('\\'))
        {
            return false;
        }

        var parts = assetPath.Split('/');
        if (parts.Length != 4
            || !ViewerStreamAddress.TryParse(
                string.Join('/', parts.Take(3)),
                out streamAddress)
            || !IsAllowedAsset(parts[3], out playlist)
            || !TryBuildQuery(query, out var queryString))
        {
            return false;
        }

        relativePath = $"{streamAddress.StreamPath}/{parts[3]}{queryString}";
        return true;
    }

    private static bool TryReadBearerToken(
        StringValues header,
        out string token)
    {
        token = string.Empty;
        if (header.Count != 1
            || !AuthenticationHeaderValue.TryParse(header[0], out var value)
            || !string.Equals(value.Scheme, "Bearer", StringComparison.OrdinalIgnoreCase)
            || string.IsNullOrWhiteSpace(value.Parameter)
            || value.Parameter.Length is < 43 or > 128
            || !value.Parameter.All(character =>
                char.IsAsciiLetterOrDigit(character)
                || character is '-' or '_'))
        {
            return false;
        }

        token = value.Parameter;
        return true;
    }

    private static bool IsAllowedAsset(string asset, out bool playlist)
    {
        playlist = asset.EndsWith(".m3u8", StringComparison.Ordinal);
        if (asset.Length is < 5 or > 180
            || (!playlist && !asset.EndsWith(".mp4", StringComparison.Ordinal)))
        {
            return false;
        }

        return asset.All(character =>
            char.IsAsciiLetterOrDigit(character)
            || character is '-' or '_' or '.');
    }

    private static bool TryBuildQuery(
        IQueryCollection query,
        out QueryString queryString)
    {
        queryString = QueryString.Empty;
        if (query.Keys.Any(key => !AllowedQueryKeys.Contains(key, StringComparer.Ordinal)))
        {
            return false;
        }

        var builder = new QueryBuilder();
        foreach (var key in AllowedQueryKeys)
        {
            if (!query.TryGetValue(key, out var values))
            {
                continue;
            }

            if (values.Count != 1 || !IsAllowedQueryValue(key, values[0]))
            {
                return false;
            }

            builder.Add(key, values[0]!);
        }

        queryString = builder.ToQueryString();
        return true;
    }

    private static bool IsAllowedQueryValue(string key, string? value)
    {
        if (string.IsNullOrEmpty(value) || value.Length > 20)
        {
            return false;
        }

        if (key == "_HLS_skip")
        {
            return value is "YES" or "v2";
        }

        return ulong.TryParse(value, out _);
    }

    private static IResult NoStore(int statusCode)
    {
        return new ViewerHlsResult(
            new ViewerHlsProxyResponse(statusCode, null, Array.Empty<byte>()));
    }

    private sealed class ViewerHlsResult : IResult
    {
        private readonly ViewerHlsProxyResponse _response;

        public ViewerHlsResult(ViewerHlsProxyResponse response)
        {
            _response = response;
        }

        public async Task ExecuteAsync(HttpContext context)
        {
            context.Response.StatusCode = _response.StatusCode;
            context.Response.Headers.CacheControl = "no-store, private";
            context.Response.Headers.Pragma = "no-cache";
            context.Response.Headers.Expires = "0";
            context.Response.Headers.Vary = "Authorization, Origin";

            if (_response.Body.Length == 0)
            {
                return;
            }

            context.Response.ContentType = _response.ContentType;
            context.Response.ContentLength = _response.Body.Length;
            await context.Response.Body.WriteAsync(
                _response.Body,
                context.RequestAborted);
        }
    }
}
