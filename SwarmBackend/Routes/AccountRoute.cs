using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using System.Security.Claims;

namespace SwarmBackend.Routes;

public static class AccountRoute
{
    public static RouteGroupBuilder MapAccount(this RouteGroupBuilder group)
    {

        group.MapPost("/authenticate", Authenticate)
            .Produces<AuthenticateResponse>();

        group.MapPost("", Create)
           .Produces<AccountResponse>();

        group.MapPost("/admin", CreateByAdmin)
            .RequireAuthorization(policy => policy.RequireRole(Role.Admin.ToString()))
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
            .Produces<AccountResponse>();

        group.MapPatch("/{accountId}", Patch)
            .RequireAuthorization()
            .Produces<AccountResponse>();

        group.MapDelete("/{accountId}", Delete)
            .RequireAuthorization()
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

    public static async Task<IResult> Create(AccountRequest request, IAccountService accountService)
    {
        var response = await accountService.Create(request);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    public static async Task<IResult> CreateByAdmin(
        AdminCreateAccountRequest request,
        IAccountService accountService)
    {
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
        var response = await accountService.Create(accountRequest, request.Role);
        return response.Match(Results.Ok, Results.BadRequest);
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

        var response = await accountService.Update(accountId, request);
        return response.Match(Results.Ok, Results.BadRequest);
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

        var response = await accountService.Update(accountId, request);
        return response.Match(Results.Ok, Results.BadRequest);
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

        var response = await accountService.Delete(accountId);
        return response ? Results.Ok() : Results.BadRequest();
    }
}
