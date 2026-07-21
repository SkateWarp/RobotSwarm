using LanguageExt.Common;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using System.Security.Claims;

namespace SwarmBackend.Routes;

public static class AccountRoute
{
    public static RouteGroupBuilder MapAccount(this RouteGroupBuilder group)
    {

        group.MapPost("/authenticate", Authenticate)
            .RequireRateLimiting(AbuseProtection.AuthenticationPolicy)
            .Produces(StatusCodes.Status429TooManyRequests)
            .Produces<AuthenticateResponse>();

        group.MapPost("", Create)
           .RequireRateLimiting(AbuseProtection.RegistrationPolicy)
           .Produces(StatusCodes.Status403Forbidden)
           .Produces(StatusCodes.Status429TooManyRequests)
           .ProducesValidationProblem()
           .Produces<AccountResponse>();

        group.MapPost("/admin", CreateByAdmin)
            .RequireAuthorization(policy => policy.RequireRole(Role.Admin.ToString()))
            .Produces(StatusCodes.Status401Unauthorized)
            .ProducesValidationProblem()
            .Produces<AccountResponse>();

        group.MapPost("/refreshToken", RefreshToken)
          .Produces<AuthenticateResponse>();

        group.MapGet("", GetAll)
            .RequireAuthorization()
            .Produces<IEnumerable<AccountResponse>>();

        group.MapGet("/{accountId}", GetById)
            .RequireAuthorization()
            .Produces<AccountResponse>();

        group.MapPut("/{accountId}", Update)
            .RequireAuthorization()
            .Produces(StatusCodes.Status401Unauthorized)
            .ProducesValidationProblem()
            .Produces<AccountResponse>();

        group.MapPatch("/{accountId}", Patch)
            .RequireAuthorization()
            .Produces(StatusCodes.Status401Unauthorized)
            .ProducesValidationProblem()
            .Produces<AccountResponse>();

        group.MapDelete("/{accountId}", Delete)
            .RequireAuthorization()
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces<bool>();
        return group;
    }

    private static int? GetAccountId(HttpContext context)
    {
        var accountIdClaim = context.User.FindFirst("id");
        return accountIdClaim != null ? int.Parse(accountIdClaim.Value) : null;
    }

    private static Role? GetRole(HttpContext context)
    {
        var roleClaim = context.User.FindFirst(ClaimTypes.Role);
        return roleClaim != null && Enum.TryParse<Role>(roleClaim.Value, out var role) ? role : null;
    }

    public static async Task<IResult> Authenticate(AuthenticateRequest request, IAccountService accountService)
    {
        var response = await accountService.Authenticate(request.Email, request.Password, null);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    internal static async Task<IResult> Create(
        AccountRequest request,
        IAccountService accountService,
        IConfiguration configuration)
    {
        if (!IsPublicRegistrationEnabled(configuration))
        {
            return Results.Problem(
                statusCode: StatusCodes.Status403Forbidden,
                title: "Public registration is disabled.",
                detail: "Ask an administrator to create the account.");
        }

        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return Results.ValidationProblem(errors);
        }

        var response = await accountService.Create(normalized);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    internal static bool IsPublicRegistrationEnabled(IConfiguration configuration)
    {
        return bool.TryParse(
                configuration["Accounts:PublicRegistrationEnabled"],
                out var enabled)
            && enabled;
    }

    public static async Task<IResult> CreateByAdmin(
        AdminCreateAccountRequest request,
        IAccountService accountService,
        HttpContext context)
    {
        var actorAccountId = GetAccountId(context);
        if (!actorAccountId.HasValue)
        {
            return Results.Unauthorized();
        }

        if (!Enum.IsDefined(request.Role))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Role)] = new[] { "Role is not supported." }
            });
        }

        var accountRequest = new AccountRequest(
            request.FirstName,
            request.LastName,
            request.Email,
            request.Password);
        if (!AccountRequestValidator.TryNormalize(
                accountRequest,
                out var normalized,
                out var errors))
        {
            return Results.ValidationProblem(errors);
        }

        var response = await accountService.CreateAuthorized(
            actorAccountId.Value,
            context.User,
            normalized,
            request.Role,
            context.RequestAborted);
        return AuthorizedResult(response, Results.Ok);
    }

    public static async Task<IResult> RefreshToken(RefreshTokenRequest request, IAccountService accountService)
    {
        var response = await accountService.RefreshTokenAsync(request.RefreshToken, null);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    public static async Task<IResult> GetAll(IAccountService accountService, HttpContext context)
    {
        var accountId = GetAccountId(context);
        var role = GetRole(context);

        if (!accountId.HasValue || !role.HasValue)
        {
            return Results.Unauthorized();
        }

        var response = await accountService.GetAll(accountId, role);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetById(int accountId, IAccountService accountService, HttpContext context)
    {
        var currentAccountId = GetAccountId(context);
        var role = GetRole(context);

        if (!currentAccountId.HasValue)
        {
            return Results.Unauthorized();
        }

        // Non-admin users can only see their own account
        if (role != Role.Admin && accountId != currentAccountId.Value)
        {
            return Results.Forbid();
        }

        var response = await accountService.GetById(accountId);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    public static async Task<IResult> Update(
        int accountId,
        AccountRequest request,
        IAccountService accountService,
        HttpContext context)
    {
        var currentAccountId = GetAccountId(context);
        var role = GetRole(context);
        if (!currentAccountId.HasValue)
        {
            return Results.Unauthorized();
        }

        if (role != Role.Admin && accountId != currentAccountId.Value)
        {
            return Results.Forbid();
        }

        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return Results.ValidationProblem(errors);
        }

        var response = await accountService.UpdateAuthorized(
            currentAccountId.Value,
            context.User,
            accountId,
            normalized,
            context.RequestAborted);
        return AuthorizedResult(response, Results.Ok);
    }

    public static async Task<IResult> Patch(
        int accountId,
        AccountPatchRequest request,
        IAccountService accountService,
        HttpContext context)
    {
        var currentAccountId = GetAccountId(context);
        var role = GetRole(context);
        if (!currentAccountId.HasValue)
        {
            return Results.Unauthorized();
        }

        if (role != Role.Admin && accountId != currentAccountId.Value)
        {
            return Results.Forbid();
        }

        if (role != Role.Admin && request.Role.HasValue)
        {
            return Results.Forbid();
        }

        if (request.Role.HasValue && !Enum.IsDefined(request.Role.Value))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Role)] = new[] { "Role is not supported." }
            });
        }

        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return Results.ValidationProblem(errors);
        }

        var response = await accountService.UpdateAuthorized(
            currentAccountId.Value,
            context.User,
            accountId,
            normalized,
            context.RequestAborted);
        return AuthorizedResult(response, Results.Ok);
    }

    public static async Task<IResult> Delete(
        int accountId,
        IAccountService accountService,
        HttpContext context)
    {
        var currentAccountId = GetAccountId(context);
        var role = GetRole(context);
        if (!currentAccountId.HasValue)
        {
            return Results.Unauthorized();
        }

        if (role != Role.Admin && accountId != currentAccountId.Value)
        {
            return Results.Forbid();
        }

        var response = await accountService.DeleteAuthorized(
            currentAccountId.Value,
            context.User,
            accountId,
            context.RequestAborted);
        return AuthorizedResult(
            response,
            deleted => deleted ? Results.Ok() : Results.BadRequest());
    }

    private static IResult AuthorizedResult<T>(
        Result<T> result,
        Func<T, IResult> success)
    {
        return result.Match(
            success,
            error => error is UnauthorizedAccessException
                ? Results.Unauthorized()
                : Results.BadRequest(error));
    }
}
