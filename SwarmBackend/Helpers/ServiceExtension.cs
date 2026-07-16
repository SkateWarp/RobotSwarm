using Microsoft.AspNetCore.Authentication;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.EntityFrameworkCore;
using Microsoft.IdentityModel.Tokens;
using Microsoft.OpenApi.Models;
using System.Security.Claims;

namespace SwarmBackend.Helpers;

public static class ServiceExtension
{
    public static void GetConfigureJwt(this IServiceCollection services, IConfiguration configuration)
    {
        var jwtConfig = configuration.GetSection("AppSettings");
        var secretValue = jwtConfig["Secret"];
        var issuer = jwtConfig["Issuer"];
        if (string.IsNullOrWhiteSpace(secretValue))
        {
            throw new InvalidOperationException("AppSettings:Secret is required.");
        }

        if (string.IsNullOrWhiteSpace(issuer)
            || !Uri.TryCreate(issuer, UriKind.Absolute, out _))
        {
            throw new InvalidOperationException("AppSettings:Issuer must be an absolute URL.");
        }

        byte[] dataDecoded;
        try
        {
            dataDecoded = Convert.FromBase64String(secretValue);
        }
        catch (FormatException exception)
        {
            throw new InvalidOperationException(
                "AppSettings:Secret must be a valid base64 value.",
                exception);
        }

        if (dataDecoded.Length < 32)
        {
            throw new InvalidOperationException(
                "AppSettings:Secret must decode to at least 32 bytes.");
        }

        services.AddAuthentication(opt =>
        {
            opt.DefaultAuthenticateScheme = JwtBearerDefaults.AuthenticationScheme;
            opt.DefaultChallengeScheme = JwtBearerDefaults.AuthenticationScheme;
        })
        .AddJwtBearer(options =>
        {
            options.TokenValidationParameters = new TokenValidationParameters
            {
                ValidateIssuer = true,
                ValidateAudience = true,
                ValidateLifetime = true,
                ValidateIssuerSigningKey = true,
                ValidIssuer = issuer,
                ValidAudience = issuer,
                IssuerSigningKey = new SymmetricSecurityKey(dataDecoded)
            };
            options.Events = new JwtBearerEvents
            {
                OnMessageReceived = context =>
                {
                    if (context.HttpContext.Request.Path.StartsWithSegments("/hubs/session")
                        && context.Request.Query.TryGetValue("access_token", out var accessToken))
                    {
                        context.Token = accessToken;
                    }

                    return Task.CompletedTask;
                },
                OnTokenValidated = async context =>
                {
                    var accountIdValue = context.Principal?.FindFirst("id")?.Value;
                    var roleValue = context.Principal?.FindFirst(ClaimTypes.Role)?.Value;
                    var versionValue = context.Principal?.FindFirst("account_version")?.Value;
                    if (!int.TryParse(accountIdValue, out var accountId)
                        || !long.TryParse(versionValue, out var accountVersion)
                        || string.IsNullOrWhiteSpace(roleValue))
                    {
                        context.Fail("The account token is missing required claims.");
                        return;
                    }

                    var dataContext = context.HttpContext.RequestServices
                        .GetRequiredService<DataContext>();
                    var account = await dataContext.Accounts
                        .AsNoTracking()
                        .SingleOrDefaultAsync(
                            candidate => candidate.Id == accountId,
                            context.HttpContext.RequestAborted);
                    var currentVersion = account?.Updated?.Ticks ?? 0L;
                    if (account == null
                        || !account.Enabled
                        || !roleValue.Equals(
                            account.Role.ToString(),
                            StringComparison.Ordinal)
                        || accountVersion != currentVersion)
                    {
                        context.Fail("The account token is no longer active.");
                    }
                }
            };
        })
        .AddScheme<AuthenticationSchemeOptions, WorkerCredentialAuthenticationHandler>(
            WorkerCredentialDefaults.AuthenticationScheme,
            _ => { });
    }


    public static void ConfigureSwagger(this IServiceCollection services)
    {
        services.AddSwaggerGen(c =>
        {
            c.SwaggerDoc("v1", new OpenApiInfo
            {
                Title = "Swarm Backend API",
                Version = "v1",
                Description = "Swarm Backend",

            });
            c.ResolveConflictingActions(apiDescriptions => apiDescriptions.First());
            c.AddSecurityDefinition("Bearer", new OpenApiSecurityScheme
            {
                Name = "Authorization",
                Type = SecuritySchemeType.ApiKey,
                Scheme = "Bearer",
                BearerFormat = "JWT",
                In = ParameterLocation.Header,
                Description = "JWT Authorization header using the Bearer scheme."

            });

            c.AddSecurityRequirement(new OpenApiSecurityRequirement
        {
            {
                new OpenApiSecurityScheme
                {
                    Reference = new OpenApiReference
                    {
                        Type = ReferenceType.SecurityScheme,
                        Id = "Bearer"
                    }
                },
                Array.Empty<string>()
            }
        });
        });
    }

}
