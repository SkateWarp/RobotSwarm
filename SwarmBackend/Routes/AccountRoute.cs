using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class AccountRoute
{
    public static RouteGroupBuilder MapAccount(this RouteGroupBuilder group)
    {

        group.MapPost("/authenticate", Authenticate)
            .Produces<AuthenticateResponse>();

        group.MapPost("", Create)
            .RequireAuthorization()
           .Produces<AccountResponse>();

        group.MapPost("/refreshToken", RefreshToken)
          .Produces<AuthenticateResponse>();

        group.MapGet("", GetAll)
      .Produces<IEnumerable<AccountResponse>>();

        return group;
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

    public static async Task<IResult> RefreshToken(RefreshTokenRequest request, IAccountService accountService)
    {
        var response = await accountService.RefreshTokenAsync(request.RefreshToken, null);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    public static async Task<IResult> GetAll(IAccountService accountService)
    {
        var response = await accountService.GetAll();
        return Results.Ok(response);
    }

}
