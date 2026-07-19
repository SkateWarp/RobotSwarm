using System.Net;
using System.Net.Http.Headers;
using System.Security.Cryptography;
using System.Text;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Primitives;
using SwarmBackend.Entities;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class ViewerHlsProxyTests
{
    private static readonly Guid TestSessionId =
        Guid.Parse("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee");
    private const string LeaseToken =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQ";
    private const string CdnSecret =
        "internal-cdn-secret-abcdefghijklmnopqrstuvwxyz01";

    [Fact]
    public void CanonicalPathKeepsOnlyLowLatencyHlsQueries()
    {
        var sessionId = Guid.Parse("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee");
        var sourceId = $"scene-{sessionId:N}";
        var query = new QueryCollection(new Dictionary<string, StringValues>
        {
            ["_HLS_part"] = "3",
            ["_HLS_msn"] = "27",
            ["_HLS_skip"] = "YES"
        });

        var valid = ViewerHlsProxyRoute.TryBuildRelativePath(
            $"session/{sessionId:N}/{sourceId}/stream.m3u8",
            query,
            out var streamAddress,
            out var relativePath,
            out var playlist);

        Assert.True(valid);
        Assert.True(playlist);
        Assert.Equal(sessionId, streamAddress.SessionId);
        Assert.Equal(
            $"session/{sessionId:N}/{sourceId}/stream.m3u8" +
            "?_HLS_msn=27&_HLS_part=3&_HLS_skip=YES",
            relativePath);
    }

    [Theory]
    [InlineData("session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.html")]
    [InlineData("session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/../index.m3u8")]
    [InlineData("session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/scene-bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb/index.m3u8")]
    [InlineData("/session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8")]
    public void RejectsNonCanonicalAssets(string assetPath)
    {
        Assert.False(ViewerHlsProxyRoute.TryBuildRelativePath(
            assetPath,
            new QueryCollection(),
            out _,
            out _,
            out _));
    }

    [Fact]
    public void RejectsUnknownOrRepeatedQueryValues()
    {
        var sessionId = Guid.Parse("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee");
        var path = $"session/{sessionId:N}/scene-{sessionId:N}/index.m3u8";

        Assert.False(ViewerHlsProxyRoute.TryBuildRelativePath(
            path,
            new QueryCollection(new Dictionary<string, StringValues>
            {
                ["token"] = "do-not-forward"
            }),
            out _,
            out _,
            out _));
        Assert.False(ViewerHlsProxyRoute.TryBuildRelativePath(
            path,
            new QueryCollection(new Dictionary<string, StringValues>
            {
                ["_HLS_msn"] = new[] { "1", "2" }
            }),
            out _,
            out _,
            out _));
    }

    [Fact]
    public async Task ProxyUsesConfiguredOriginAndSeparateCdnCredential()
    {
        Uri? requestedUri = null;
        AuthenticationHeaderValue? authorization = null;
        CacheControlHeaderValue? cacheControl = null;
        var handler = new StubHandler(request =>
        {
            requestedUri = request.RequestUri;
            authorization = request.Headers.Authorization;
            cacheControl = request.Headers.CacheControl;
            return new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent("#EXTM3U\n")
            };
        });
        var proxy = CreateProxy(handler);

        var response = await proxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status200OK, response.StatusCode);
        Assert.Equal("#EXTM3U\n", Encoding.UTF8.GetString(response.Body));
        Assert.Equal(
            "http://media:8888/session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            requestedUri?.ToString());
        Assert.Equal("Bearer", authorization?.Scheme);
        Assert.Equal(CdnSecret, authorization?.Parameter);
        Assert.NotEqual(LeaseToken, authorization?.Parameter);
        Assert.True(cacheControl?.NoCache);
        Assert.True(cacheControl?.NoStore);
    }

    [Fact]
    public async Task ProxyRejectsRedirectsAndOversizedResponses()
    {
        var redirectProxy = CreateProxy(new StubHandler(_ =>
        {
            var response = new HttpResponseMessage(HttpStatusCode.Redirect);
            response.Headers.Location = new Uri("http://untrusted.example/stream.m3u8");
            return response;
        }));
        var oversizedProxy = CreateProxy(new StubHandler(_ =>
            new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new ByteArrayContent(
                    new byte[ViewerHlsProxy.MaxPlaylistBytes + 1])
            }));

        var redirect = await redirectProxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);
        var oversized = await oversizedProxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status502BadGateway, redirect.StatusCode);
        Assert.Equal(StatusCodes.Status502BadGateway, oversized.StatusCode);
    }

    [Fact]
    public async Task UpstreamAuthFailureIsReportedAsAProxyFailure()
    {
        var proxy = CreateProxy(new StubHandler(_ =>
            new HttpResponseMessage(HttpStatusCode.Unauthorized)));

        var response = await proxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status502BadGateway, response.StatusCode);
    }

    [Fact]
    public async Task InterruptedUpstreamBodyIsReportedAsAProxyFailure()
    {
        var proxy = CreateProxy(new StubHandler(_ =>
            new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StreamContent(new InterruptedReadStream())
            }));

        var response = await proxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status502BadGateway, response.StatusCode);
    }

    [Fact]
    public async Task ProxyFailsClosedWithoutAValidInternalCredential()
    {
        var upstreamCalls = 0;
        var configuration = new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Viewer:HlsOriginBaseUrl"] = "http://media:8888",
                ["Viewer:HlsCdnSecret"] = "too-short"
            })
            .Build();
        var proxy = new ViewerHlsProxy(
            new HttpClient(new StubHandler(_ =>
            {
                upstreamCalls += 1;
                return new HttpResponseMessage(HttpStatusCode.OK);
            })),
            configuration,
            new ViewerHlsConcurrencyGate());

        var response = await proxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.False(proxy.IsConfigured);
        Assert.Equal(StatusCodes.Status503ServiceUnavailable, response.StatusCode);
        Assert.Equal(0, upstreamCalls);
    }

    [Fact]
    public async Task ProxyRejectsExcessConcurrentRequestsBeforeOpeningUpstream()
    {
        var upstreamCalls = 0;
        var gate = new ViewerHlsConcurrencyGate(
            globalPermitLimit: 1,
            perSessionPermitLimit: 1);
        using var occupiedPermit = gate.TryEnter(TestSessionId);
        var proxy = new ViewerHlsProxy(
            new HttpClient(new StubHandler(_ =>
            {
                upstreamCalls += 1;
                return new HttpResponseMessage(HttpStatusCode.OK);
            })),
            Configuration(enabled: true),
            gate);

        var response = await proxy.Fetch(
            "session/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/" +
            "scene-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/index.m3u8",
            TestSessionId,
            playlist: true,
            CancellationToken.None);

        Assert.NotNull(occupiedPermit);
        Assert.Equal(StatusCodes.Status429TooManyRequests, response.StatusCode);
        Assert.Equal(0, upstreamCalls);
    }

    [Fact]
    public void ConcurrencyGateDoesNotMakeUnrelatedSessionsShareAQuota()
    {
        var firstSession = Guid.Empty;
        var secondSession = new Guid(64, 0, 0, new byte[8]);
        Assert.Equal(
            (uint)firstSession.GetHashCode() % 64,
            (uint)secondSession.GetHashCode() % 64);

        var gate = new ViewerHlsConcurrencyGate(
            globalPermitLimit: 2,
            perSessionPermitLimit: 1);
        using var firstPermit = gate.TryEnter(firstSession);
        using var secondPermit = gate.TryEnter(secondSession);

        Assert.NotNull(firstPermit);
        Assert.NotNull(secondPermit);
        Assert.Null(gate.TryEnter(firstSession));
    }

    [Fact]
    public async Task RouteValidatesTheLeaseBeforeCallingMediaMtx()
    {
        await using var dataContext = TestDataContext.Create();
        var account = CreateAccount();
        var session = CreateSession(account);
        dataContext.AddRange(account, session, CreateLease(session, account));
        await dataContext.SaveChangesAsync();

        var upstreamCalls = 0;
        var proxy = CreateProxy(new StubHandler(_ =>
        {
            upstreamCalls += 1;
            return new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent("#EXTM3U\n")
            };
        }));
        var configuration = Configuration(enabled: true);
        var wrongTokenContext = RequestContext("wrong-token-that-is-long-enough-0000000000000");

        var rejected = await ViewerHlsProxyRoute.Proxy(
            SceneAsset(session.Id),
            wrongTokenContext,
            dataContext,
            configuration,
            proxy,
            CancellationToken.None);
        await rejected.ExecuteAsync(wrongTokenContext);

        Assert.Equal(StatusCodes.Status401Unauthorized, wrongTokenContext.Response.StatusCode);
        Assert.Equal(0, upstreamCalls);

        var validContext = RequestContext(LeaseToken);
        var accepted = await ViewerHlsProxyRoute.Proxy(
            SceneAsset(session.Id),
            validContext,
            dataContext,
            configuration,
            proxy,
            CancellationToken.None);
        await accepted.ExecuteAsync(validContext);

        Assert.Equal(StatusCodes.Status200OK, validContext.Response.StatusCode);
        Assert.Equal("no-store, private", validContext.Response.Headers.CacheControl);
        Assert.Contains("Authorization", validContext.Response.Headers.Vary.ToString());
        Assert.Equal(1, upstreamCalls);
    }

    [Fact]
    public async Task DisabledRouteDoesNotValidateOrContactUpstream()
    {
        await using var dataContext = TestDataContext.Create();
        var upstreamCalls = 0;
        var proxy = CreateProxy(new StubHandler(_ =>
        {
            upstreamCalls += 1;
            return new HttpResponseMessage(HttpStatusCode.OK);
        }));
        var context = RequestContext(LeaseToken);

        var result = await ViewerHlsProxyRoute.Proxy(
            SceneAsset(Guid.NewGuid()),
            context,
            dataContext,
            Configuration(enabled: false),
            proxy,
            CancellationToken.None);
        await result.ExecuteAsync(context);

        Assert.Equal(StatusCodes.Status404NotFound, context.Response.StatusCode);
        Assert.Equal("no-store, private", context.Response.Headers.CacheControl);
        Assert.Equal(0, upstreamCalls);
    }

    private static ViewerHlsProxy CreateProxy(HttpMessageHandler handler)
    {
        return new ViewerHlsProxy(
            new HttpClient(handler),
            Configuration(enabled: true),
            new ViewerHlsConcurrencyGate());
    }

    private static IConfiguration Configuration(bool enabled)
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Viewer:HlsProxyEnabled"] = enabled.ToString(),
                ["Viewer:HlsOriginBaseUrl"] = "http://media:8888",
                ["Viewer:HlsCdnSecret"] = CdnSecret,
                ["Viewer:HlsUpstreamTimeoutSeconds"] = "5"
            })
            .Build();
    }

    private static DefaultHttpContext RequestContext(string token)
    {
        var context = new DefaultHttpContext();
        context.Request.Headers.Authorization = $"Bearer {token}";
        context.Response.Body = new MemoryStream();
        return context;
    }

    private static Account CreateAccount()
    {
        return new Account
        {
            Id = 41,
            FirstName = "HLS",
            LastName = "Viewer",
            Email = "hls-viewer@example.test",
            Enabled = true
        };
    }

    private static SimulationSession CreateSession(Account account)
    {
        return new SimulationSession
        {
            AccountId = account.Id,
            Account = account,
            State = SimulationSessionState.Active,
            DesiredRobotCount = 1
        };
    }

    private static ViewerLease CreateLease(
        SimulationSession session,
        Account account)
    {
        return new ViewerLease
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            AccountId = account.Id,
            Account = account,
            Source = ViewerSourceType.Scene,
            TokenHash = Convert.ToHexString(
                SHA256.HashData(Encoding.UTF8.GetBytes(LeaseToken))),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        };
    }

    private static string SceneAsset(Guid sessionId)
    {
        return $"session/{sessionId:N}/scene-{sessionId:N}/index.m3u8";
    }

    private sealed class StubHandler : HttpMessageHandler
    {
        private readonly Func<HttpRequestMessage, HttpResponseMessage> _response;

        public StubHandler(Func<HttpRequestMessage, HttpResponseMessage> response)
        {
            _response = response;
        }

        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            return Task.FromResult(_response(request));
        }
    }

    private sealed class InterruptedReadStream : Stream
    {
        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => false;
        public override long Length => throw new NotSupportedException();
        public override long Position
        {
            get => throw new NotSupportedException();
            set => throw new NotSupportedException();
        }

        public override int Read(byte[] buffer, int offset, int count)
        {
            throw new IOException("The upstream closed the media response.");
        }

        public override ValueTask<int> ReadAsync(
            Memory<byte> buffer,
            CancellationToken cancellationToken = default)
        {
            return ValueTask.FromException<int>(
                new IOException("The upstream closed the media response."));
        }

        public override void Flush()
        {
        }

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotSupportedException();
        }

        public override void SetLength(long value)
        {
            throw new NotSupportedException();
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            throw new NotSupportedException();
        }
    }
}
