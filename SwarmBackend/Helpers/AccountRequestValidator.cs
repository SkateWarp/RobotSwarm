using System.Net.Mail;
using System.Text.RegularExpressions;
using SwarmBackend.Models;

namespace SwarmBackend.Helpers;

public static class AccountRequestValidator
{
    public const int MinimumPasswordLength = 8;
    public const int MaximumPasswordLength = 16;
    public const int MaximumEmailLength = 254;
    public const string CanonicalEmailSql =
        "lower(btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)))";
    public const string SupportedEmailCheckSql =
        "char_length(btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13))) BETWEEN 1 AND 254 "
        + "AND btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)) "
        + "~ '^[A-Za-z0-9._%+-]+@([A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?\\.)+[A-Za-z]{2,63}$'";

    private static readonly char[] CanonicalEmailWhitespace =
        [' ', '\t', '\n', '\v', '\f', '\r'];

    private static readonly Regex EmailPattern = new(
        @"^[A-Za-z0-9._%+-]+@(?:[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)+[A-Za-z]{2,63}$",
        RegexOptions.CultureInvariant | RegexOptions.Compiled);
    private static readonly Regex PasswordPattern = new(
        @"^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$",
        RegexOptions.CultureInvariant | RegexOptions.Compiled);

    public static string CanonicalEmail(string email)
    {
        return email.Trim(CanonicalEmailWhitespace).ToLowerInvariant();
    }

    public static bool TryCanonicalizeEmail(string? value, out string normalized)
    {
        var errors = new Dictionary<string, string[]>();
        normalized = NormalizeEmail(value, required: true, errors);
        return errors.Count == 0;
    }

    public static bool TryNormalize(
        AccountRequest request,
        out AccountRequest normalized,
        out Dictionary<string, string[]> errors)
    {
        errors = new Dictionary<string, string[]>();
        var firstName = NormalizeRequiredName(request.FirstName, nameof(request.FirstName), errors);
        var lastName = NormalizeRequiredName(request.LastName, nameof(request.LastName), errors);
        var email = NormalizeEmail(request.Email, required: true, errors);
        ValidatePassword(request.Password, required: true, errors);

        normalized = new AccountRequest(
            firstName,
            lastName,
            email,
            request.Password ?? string.Empty);
        return errors.Count == 0;
    }

    public static bool TryNormalize(
        AccountPatchRequest request,
        out AccountPatchRequest normalized,
        out Dictionary<string, string[]> errors)
    {
        errors = new Dictionary<string, string[]>();
        var firstName = request.FirstName == null
            ? null
            : NormalizeRequiredName(request.FirstName, nameof(request.FirstName), errors);
        var lastName = request.LastName == null
            ? null
            : NormalizeRequiredName(request.LastName, nameof(request.LastName), errors);
        var email = request.Email == null
            ? null
            : NormalizeEmail(request.Email, required: true, errors);
        if (request.Password != null)
        {
            ValidatePassword(request.Password, required: true, errors);
        }
        if (request.Role.HasValue && !Enum.IsDefined(request.Role.Value))
        {
            errors[nameof(AccountPatchRequest.Role)] = ["Role is not supported."];
        }

        normalized = new AccountPatchRequest(
            firstName,
            lastName,
            email,
            request.Password,
            request.Role);
        return errors.Count == 0;
    }

    private static string NormalizeRequiredName(
        string? value,
        string field,
        Dictionary<string, string[]> errors)
    {
        var normalized = value?.Trim() ?? string.Empty;
        if (normalized.Length == 0)
        {
            errors[field] = ["El campo es obligatorio."];
        }

        return normalized;
    }

    private static string NormalizeEmail(
        string? value,
        bool required,
        Dictionary<string, string[]> errors)
    {
        var normalized = value == null ? string.Empty : CanonicalEmail(value);
        if (normalized.Length == 0)
        {
            if (required)
            {
                errors[nameof(AccountRequest.Email)] = ["El correo electrónico es obligatorio."];
            }
            return normalized;
        }

        if (normalized.Length > MaximumEmailLength
            || !MailAddress.TryCreate(normalized, out var address)
            || !address.Address.Equals(normalized, StringComparison.OrdinalIgnoreCase)
            || !EmailPattern.IsMatch(normalized))
        {
            errors[nameof(AccountRequest.Email)] = ["Ingrese un correo electrónico válido."];
        }

        return normalized;
    }

    private static void ValidatePassword(
        string? password,
        bool required,
        Dictionary<string, string[]> errors)
    {
        if (password == null)
        {
            if (required)
            {
                errors[nameof(AccountRequest.Password)] = ["La contraseña es obligatoria."];
            }
            return;
        }

        if (password.Length < MinimumPasswordLength
            || password.Length > MaximumPasswordLength)
        {
            errors[nameof(AccountRequest.Password)] =
                [$"La contraseña debe tener entre {MinimumPasswordLength} y {MaximumPasswordLength} caracteres."];
            return;
        }

        if (!PasswordPattern.IsMatch(password))
        {
            errors[nameof(AccountRequest.Password)] =
                ["La contraseña debe incluir una minúscula, un número y un símbolo !@#$%^&*, sin otros caracteres."];
        }
    }
}
