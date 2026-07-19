using System.Buffers;
using System.Net;
using System.Net.Http.Headers;

namespace SwarmBackend.Services;

public sealed record ViewerHlsProxyResponse(
    int StatusCode,
    string? ContentType,
    byte[] Body);

public sealed class ViewerHlsProxy
{
    internal const int MaxPlaylistBytes = 512 * 1024;
    internal const int MaxMediaBytes = 12 * 1024 * 1024;
    internal const int MaximumConcurrentRequests = 12;

    private readonly HttpClient _httpClient;
    private readonly Uri? _origin;
    private readonly string? _cdnSecret;
    private readonly TimeSpan _requestTimeout;
    private readonly ViewerHlsConcurrencyGate _concurrencyGate;

    public ViewerHlsProxy(
        HttpClient httpClient,
        IConfiguration configuration,
        ViewerHlsConcurrencyGate concurrencyGate)
    {
        _httpClient = httpClient;
        _concurrencyGate = concurrencyGate;
        _origin = ParseOrigin(configuration["Viewer:HlsOriginBaseUrl"]);
        _cdnSecret = ParseCdnSecret(configuration["Viewer:HlsCdnSecret"]);

        var timeoutSeconds = configuration.GetValue<int?>(
            "Viewer:HlsUpstreamTimeoutSeconds") ?? 15;
        _requestTimeout = TimeSpan.FromSeconds(Math.Clamp(timeoutSeconds, 5, 30));
    }

    public bool IsConfigured => _origin != null && _cdnSecret != null;

    public async Task<ViewerHlsProxyResponse> Fetch(
        string relativePath,
        Guid sessionId,
        bool playlist,
        CancellationToken cancellationToken)
    {
        if (_origin == null || _cdnSecret == null)
        {
            return Empty(StatusCodes.Status503ServiceUnavailable);
        }

        using var permit = _concurrencyGate.TryEnter(sessionId);
        if (permit == null)
        {
            return Empty(StatusCodes.Status429TooManyRequests);
        }

        using var request = new HttpRequestMessage(
            HttpMethod.Get,
            new Uri(_origin, relativePath));
        request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", _cdnSecret);
        request.Headers.CacheControl = new CacheControlHeaderValue
        {
            NoCache = true,
            NoStore = true
        };
        request.Headers.Accept.Add(new MediaTypeWithQualityHeaderValue(
            playlist ? "application/vnd.apple.mpegurl" : "video/mp4"));

        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(_requestTimeout);

        try
        {
            using var response = await _httpClient.SendAsync(
                request,
                HttpCompletionOption.ResponseHeadersRead,
                timeout.Token);

            if (response.StatusCode == HttpStatusCode.Unauthorized
                || response.StatusCode == HttpStatusCode.Forbidden)
            {
                // The browser lease was already checked by the route. An auth
                // failure here means the private CDN credential is out of sync.
                return Empty(StatusCodes.Status502BadGateway);
            }

            if (response.StatusCode == HttpStatusCode.NotFound)
            {
                return Empty(StatusCodes.Status404NotFound);
            }

            if (response.StatusCode != HttpStatusCode.OK)
            {
                return Empty(StatusCodes.Status502BadGateway);
            }

            var limit = playlist ? MaxPlaylistBytes : MaxMediaBytes;
            if (response.Content.Headers.ContentLength > limit)
            {
                return Empty(StatusCodes.Status502BadGateway);
            }

            var body = await ReadBounded(response.Content, limit, timeout.Token);
            if (body == null)
            {
                return Empty(StatusCodes.Status502BadGateway);
            }

            return new ViewerHlsProxyResponse(
                StatusCodes.Status200OK,
                playlist ? "application/vnd.apple.mpegurl" : "video/mp4",
                body);
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return Empty(StatusCodes.Status504GatewayTimeout);
        }
        catch (HttpRequestException)
        {
            return Empty(StatusCodes.Status502BadGateway);
        }
        catch (IOException)
        {
            return Empty(StatusCodes.Status502BadGateway);
        }
    }

    private static Uri? ParseOrigin(string? value)
    {
        if (string.IsNullOrWhiteSpace(value)
            || !Uri.TryCreate(value, UriKind.Absolute, out var origin)
            || origin.Scheme is not ("http" or "https")
            || string.IsNullOrWhiteSpace(origin.Host)
            || !string.IsNullOrEmpty(origin.UserInfo)
            || !string.IsNullOrEmpty(origin.Query)
            || !string.IsNullOrEmpty(origin.Fragment))
        {
            return null;
        }

        return new Uri(value.Trim().TrimEnd('/') + "/", UriKind.Absolute);
    }

    private static string? ParseCdnSecret(string? value)
    {
        if (string.IsNullOrWhiteSpace(value)
            || value.Length is < 43 or > 128
            || !value.All(character =>
                char.IsAsciiLetterOrDigit(character)
                || character is '-' or '_'))
        {
            return null;
        }

        return value;
    }

    private static async Task<byte[]?> ReadBounded(
        HttpContent content,
        int limit,
        CancellationToken cancellationToken)
    {
        await using var input = await content.ReadAsStreamAsync(cancellationToken);
        using var output = new MemoryStream();
        var buffer = ArrayPool<byte>.Shared.Rent(64 * 1024);
        try
        {
            while (true)
            {
                var count = await input.ReadAsync(buffer, cancellationToken);
                if (count == 0)
                {
                    return output.ToArray();
                }

                if (output.Length + count > limit)
                {
                    return null;
                }

                await output.WriteAsync(buffer.AsMemory(0, count), cancellationToken);
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(buffer, clearArray: true);
        }
    }

    private static ViewerHlsProxyResponse Empty(int statusCode)
    {
        return new ViewerHlsProxyResponse(statusCode, null, Array.Empty<byte>());
    }
}

public sealed class ViewerHlsConcurrencyGate
{
    private const int PerSessionPermitLimit = 4;

    private readonly object _sync = new();
    private readonly int _globalPermitLimit;
    private readonly int _perSessionPermitLimit;
    private readonly Dictionary<Guid, int> _sessionUse = new();
    private int _globalUse;

    public ViewerHlsConcurrencyGate()
        : this(
            ViewerHlsProxy.MaximumConcurrentRequests,
            PerSessionPermitLimit)
    {
    }

    internal ViewerHlsConcurrencyGate(
        int globalPermitLimit,
        int perSessionPermitLimit)
    {
        _globalPermitLimit = globalPermitLimit;
        _perSessionPermitLimit = perSessionPermitLimit;
    }

    internal IDisposable? TryEnter(Guid sessionId)
    {
        lock (_sync)
        {
            _sessionUse.TryGetValue(sessionId, out var sessionUse);
            if (_globalUse >= _globalPermitLimit
                || sessionUse >= _perSessionPermitLimit)
            {
                return null;
            }

            _globalUse++;
            _sessionUse[sessionId] = sessionUse + 1;
            return new Permit(this, sessionId);
        }
    }

    private sealed class Permit(
        ViewerHlsConcurrencyGate owner,
        Guid sessionId) : IDisposable
    {
        private int _released;

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _released, 1) == 0)
            {
                owner.Release(sessionId);
            }
        }
    }

    private void Release(Guid sessionId)
    {
        lock (_sync)
        {
            var remaining = _sessionUse[sessionId] - 1;
            if (remaining == 0)
            {
                _sessionUse.Remove(sessionId);
            }
            else
            {
                _sessionUse[sessionId] = remaining;
            }

            _globalUse--;
        }
    }
}

public static class ViewerHlsServiceCollectionExtensions
{
    public static IServiceCollection AddViewerHlsProxy(this IServiceCollection services)
    {
        services.AddSingleton<ViewerHlsConcurrencyGate>();
        services
            .AddHttpClient<ViewerHlsProxy>(client =>
            {
                client.Timeout = Timeout.InfiniteTimeSpan;
            })
            .ConfigurePrimaryHttpMessageHandler(() => new SocketsHttpHandler
            {
                AllowAutoRedirect = false,
                AutomaticDecompression = DecompressionMethods.None,
                ConnectTimeout = TimeSpan.FromSeconds(3),
                MaxConnectionsPerServer = ViewerHlsProxy.MaximumConcurrentRequests,
                PooledConnectionLifetime = TimeSpan.FromMinutes(5),
                UseCookies = false,
                UseProxy = false
            });
        return services;
    }
}
