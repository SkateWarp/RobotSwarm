using System.Security.Cryptography;
using System.Text;
using Microsoft.AspNetCore.WebUtilities;

namespace SwarmBackend.Helpers;

public static class ViewerPublishToken
{
    private const int TokenBytes = 32;

    public static (string Token, string Hash) Generate()
    {
        var token = WebEncoders.Base64UrlEncode(
            RandomNumberGenerator.GetBytes(TokenBytes));
        return (token, Hash(token));
    }

    public static string Hash(string token)
    {
        return Convert.ToHexString(
            SHA256.HashData(Encoding.UTF8.GetBytes(token)));
    }

    public static bool TryHash(string token, out string hash)
    {
        hash = string.Empty;
        if (string.IsNullOrWhiteSpace(token))
        {
            return false;
        }

        try
        {
            var decoded = WebEncoders.Base64UrlDecode(token);
            if (decoded.Length != TokenBytes
                || !WebEncoders.Base64UrlEncode(decoded).Equals(
                    token,
                    StringComparison.Ordinal))
            {
                return false;
            }

            hash = Hash(token);
            return true;
        }
        catch (FormatException)
        {
            return false;
        }
    }
}
