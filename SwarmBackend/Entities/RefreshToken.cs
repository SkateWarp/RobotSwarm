using Microsoft.EntityFrameworkCore;

namespace SwarmBackend.Entities;

[Owned]
public class RefreshToken
{
    /// <summary>
    ///
    /// </summary>
    public int Id { get; set; }
    /// <summary>
    ///
    /// </summary>
    public virtual Account? Account { get; set; }
    /// <summary>
    ///
    /// </summary>
    public string Token { get; set; } = null!;
    /// <summary>
    ///
    /// </summary>
    public DateTime Expires { get; set; }
    /// <summary>
    ///
    /// </summary>
    private bool IsExpired => DateTime.Now >= Expires;
    /// <summary>
    ///
    /// </summary>
    public DateTime Created { get; set; } = DateTime.Now;
    /// <summary>
    ///
    /// </summary>
    public string CreatedByIp { get; set; } = null!;
    /// <summary>
    ///
    /// </summary>
    public DateTime? Revoked { get; set; }
    /// <summary>
    ///
    /// </summary>
    public string? RevokedByIp { get; set; }
    /// <summary>
    ///
    /// </summary>
    public string? ReplacedByToken { get; set; }
    /// <summary>
    ///
    /// </summary>
    public bool IsActive => Revoked == null && !IsExpired;

}
