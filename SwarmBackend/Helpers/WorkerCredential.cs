using System.Security.Cryptography;
using System.Text;
using Microsoft.AspNetCore.WebUtilities;

namespace SwarmBackend.Helpers;

public static class WorkerCredential
{
    public static (string Secret, string Hash) Generate()
    {
        var secret = WebEncoders.Base64UrlEncode(RandomNumberGenerator.GetBytes(32));
        return (secret, Hash(secret));
    }

    public static string Hash(string secret)
    {
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(secret)));
    }

    public static bool Verify(string secret, string expectedHash)
    {
        try
        {
            return CryptographicOperations.FixedTimeEquals(
                Convert.FromHexString(Hash(secret)),
                Convert.FromHexString(expectedHash));
        }
        catch (FormatException)
        {
            return false;
        }
    }

    public static bool TryParse(string credential, out Guid workerId, out string secret)
    {
        var separatorIndex = credential.IndexOf('.');
        if (separatorIndex <= 0 || separatorIndex == credential.Length - 1)
        {
            workerId = Guid.Empty;
            secret = string.Empty;
            return false;
        }

        secret = credential[(separatorIndex + 1)..];
        return Guid.TryParse(credential[..separatorIndex], out workerId);
    }
}
