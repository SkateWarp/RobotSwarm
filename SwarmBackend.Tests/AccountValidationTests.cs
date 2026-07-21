using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class AccountValidationTests
{
    [Fact]
    public void CanonicalEmailUsesOnlyTheExplicitAsciiBoundaryWhitespace()
    {
        Assert.Equal(
            "user@example.test",
            AccountRequestValidator.CanonicalEmail(
                " \t\n\v\f\rUser@Example.Test\r\f\v\n\t "));
        Assert.Equal(
            "\u00a0user@example.test\u00a0",
            AccountRequestValidator.CanonicalEmail(
                "\u00a0User@Example.Test\u00a0"));
    }

    [Theory]
    [InlineData("Short1!")]
    [InlineData("aaaaaaaaaaaaaaa1!")]
    [InlineData("ABCDEFG1!")]
    [InlineData("abcdefgh!")]
    [InlineData("abcdefgh1")]
    [InlineData("abcdef1!-")]
    public void CompleteRequestRejectsPasswordOutsideTheFrontendPolicy(string password)
    {
        var valid = AccountRequestValidator.TryNormalize(
            new AccountRequest("Ada", "Lovelace", "ada@example.test", password),
            out _,
            out var errors);

        Assert.False(valid);
        Assert.Contains(nameof(AccountRequest.Password), errors.Keys);
    }

    [Theory]
    [InlineData("Valid1!a")]
    [InlineData("password1!")]
    public void CompleteRequestAcceptsTheExactFrontendPasswordPolicy(string password)
    {
        var valid = AccountRequestValidator.TryNormalize(
            new AccountRequest("Ada", "Lovelace", "ada@example.test", password),
            out var normalized,
            out var errors);

        Assert.True(valid);
        Assert.Empty(errors);
        Assert.Equal(password, normalized.Password);
    }

    [Theory]
    [InlineData("")]
    [InlineData("not-an-email")]
    [InlineData("user@-example.test")]
    [InlineData("user@example..test")]
    [InlineData("\u00a0user@example.test\u00a0")]
    [InlineData("usér@example.test")]
    public void CompleteRequestRejectsInvalidEmail(string email)
    {
        var valid = AccountRequestValidator.TryNormalize(
            new AccountRequest("Ada", "Lovelace", email, "Valid1!a"),
            out _,
            out var errors);

        Assert.False(valid);
        Assert.Contains(nameof(AccountRequest.Email), errors.Keys);
    }

    [Fact]
    public void PatchLeavesNullPasswordAloneButRejectsAnEmptyPassword()
    {
        Assert.True(AccountRequestValidator.TryNormalize(
            new AccountPatchRequest(null, null, null, null, null),
            out _,
            out var noChangeErrors));
        Assert.Empty(noChangeErrors);

        Assert.False(AccountRequestValidator.TryNormalize(
            new AccountPatchRequest(null, null, null, string.Empty, null),
            out _,
            out var emptyPasswordErrors));
        Assert.Contains(nameof(AccountRequest.Password), emptyPasswordErrors.Keys);
    }

    [Fact]
    public async Task CreateCanonicalizesFieldsAndRejectsCaseInsensitiveDuplicateEmail()
    {
        await using var context = TestDataContext.Create();
        var service = new AccountService(context, Configuration());

        var first = await service.Create(new AccountRequest(
            "  Ada ",
            " Lovelace  ",
            "\t\r\n  Ada@Example.Test \v\f",
            "Valid1!a"));
        var duplicate = await service.Create(new AccountRequest(
            "Other",
            "Person",
            "ADA@example.test",
            "Other2@a"));

        Assert.True(Succeeded(first));
        Assert.False(Succeeded(duplicate));
        var account = Assert.Single(await context.Accounts.ToListAsync());
        Assert.Equal("Ada", account.FirstName);
        Assert.Equal("Lovelace", account.LastName);
        Assert.Equal("ada@example.test", account.Email);
        Assert.True(BCrypt.Net.BCrypt.Verify("Valid1!a", account.PasswordHash));
    }

    [Fact]
    public async Task CreateRejectsInvalidDataWithoutWritingAnAccount()
    {
        await using var context = TestDataContext.Create();

        var result = await new AccountService(context, Configuration()).Create(
            new AccountRequest("Ada", "Lovelace", "not-an-email", "short"));

        Assert.False(Succeeded(result));
        Assert.Empty(await context.Accounts.ToListAsync());
    }

    [Fact]
    public async Task PutRejectsInvalidAndDuplicateDataWithoutChangingTheAccount()
    {
        await using var context = TestDataContext.Create();
        var first = Account("first@example.test", "first-hash");
        var second = Account("second@example.test", "second-hash");
        context.Accounts.AddRange(first, second);
        await context.SaveChangesAsync();
        var service = new AccountService(context, Configuration());

        var invalid = await service.Update(first.Id, new AccountRequest(
            "Changed",
            "Invalid",
            "not-an-email",
            "illegal-pass"));
        var duplicate = await service.Update(first.Id, new AccountRequest(
            "Changed",
            "Duplicate",
            " SECOND@EXAMPLE.TEST ",
            "Changed1!"));

        Assert.False(Succeeded(invalid));
        Assert.False(Succeeded(duplicate));
        context.ChangeTracker.Clear();
        var unchanged = await context.Accounts.SingleAsync(account => account.Id == first.Id);
        Assert.Equal("First", unchanged.FirstName);
        Assert.Equal("Account", unchanged.LastName);
        Assert.Equal("first@example.test", unchanged.Email);
        Assert.Equal("first-hash", unchanged.PasswordHash);
    }

    [Fact]
    public async Task PatchCanonicalizesEmailWithoutReplacingAnOmittedPassword()
    {
        await using var context = TestDataContext.Create();
        var account = Account("old@example.test", "existing-hash");
        context.Accounts.Add(account);
        await context.SaveChangesAsync();
        var service = new AccountService(context, Configuration());

        var result = await service.Update(account.Id, new AccountPatchRequest(
            "  Updated ",
            null,
            " NEW@Example.Test ",
            null,
            null));

        Assert.True(Succeeded(result));
        context.ChangeTracker.Clear();
        var updated = await context.Accounts.SingleAsync(candidate => candidate.Id == account.Id);
        Assert.Equal("Updated", updated.FirstName);
        Assert.Equal("new@example.test", updated.Email);
        Assert.Equal("existing-hash", updated.PasswordHash);
        Assert.NotNull(updated.Updated);
    }

    [Fact]
    public async Task PatchRejectsDuplicateEmailWithoutChangingTheAccount()
    {
        await using var context = TestDataContext.Create();
        var first = Account("first@example.test", "first-hash");
        var second = Account("second@example.test", "second-hash");
        context.Accounts.AddRange(first, second);
        await context.SaveChangesAsync();

        var result = await new AccountService(context, Configuration()).Update(
            first.Id,
            new AccountPatchRequest(null, null, " Second@Example.Test ", null, null));

        Assert.False(Succeeded(result));
        context.ChangeTracker.Clear();
        var unchanged = await context.Accounts.SingleAsync(account => account.Id == first.Id);
        Assert.Equal("first@example.test", unchanged.Email);
        Assert.Equal("first-hash", unchanged.PasswordHash);
    }

    [Fact]
    public async Task PatchRejectsInvalidDataWithoutChangingTheAccount()
    {
        await using var context = TestDataContext.Create();
        var account = Account("first@example.test", "first-hash");
        context.Accounts.Add(account);
        await context.SaveChangesAsync();

        var result = await new AccountService(context, Configuration()).Update(
            account.Id,
            new AccountPatchRequest(null, null, "not-an-email", string.Empty, null));

        Assert.False(Succeeded(result));
        context.ChangeTracker.Clear();
        var unchanged = await context.Accounts.SingleAsync(candidate => candidate.Id == account.Id);
        Assert.Equal("first@example.test", unchanged.Email);
        Assert.Equal("first-hash", unchanged.PasswordHash);
    }

    [Fact]
    public async Task LegacyMixedCaseEmailStillAuthenticatesWithoutBeingRenamed()
    {
        await using var context = TestDataContext.Create();
        var account = Account(
            "\t\r\n Legacy.User@Example.Test \v\f",
            BCrypt.Net.BCrypt.HashPassword("Valid1!a"));
        context.Accounts.Add(account);
        await context.SaveChangesAsync();

        var result = await new AccountService(context, Configuration()).Authenticate(
            "\flegacy.user@example.test\r\n",
            "Valid1!a",
            "127.0.0.1");

        Assert.True(Succeeded(result));
        context.ChangeTracker.Clear();
        var unchanged = await context.Accounts.SingleAsync(candidate => candidate.Id == account.Id);
        Assert.Equal("\t\r\n Legacy.User@Example.Test \v\f", unchanged.Email);
    }

    private static Account Account(string email, string passwordHash)
    {
        var now = DateTime.UtcNow;
        return new Account
        {
            FirstName = "First",
            LastName = "Account",
            Email = email,
            PasswordHash = passwordHash,
            Enabled = true,
            Verified = now,
            Created = now,
            DateCreated = now,
            Role = Role.User
        };
    }

    private static bool Succeeded<T>(Result<T> result)
    {
        return result.Match(_ => true, _ => false);
    }

    private static IConfiguration Configuration()
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["AppSettings:Secret"] = Convert.ToBase64String(new byte[32]),
                ["AppSettings:Issuer"] = "robotswarm-tests",
                ["AppSettings:ExpiresIn"] = "15",
                ["AppSettings:RefreshTokenExpiresInDays"] = "7"
            })
            .Build();
    }
}
