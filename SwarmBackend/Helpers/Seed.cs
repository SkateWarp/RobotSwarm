using SwarmBackend.Models;

namespace SwarmBackend.Helpers
{
    public class Seed
    {

        public static AccountRequest GetAccount() => new(
        "admin",
        "admin",
            "admin@admin.com",
            "admin");
    }
}
