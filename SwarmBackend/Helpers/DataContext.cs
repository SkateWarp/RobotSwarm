using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public class DataContext : DbContext
{

    public DataContext(DbContextOptions<DataContext> options) : base(options)
    {
        NpgsqlConnection.GlobalTypeMapper.MapEnum<SensorTypeEnum>();
    }

    public DbSet<Account> Accounts => Set<Account>();
    public DbSet<RefreshToken> RefreshTokens => Set<RefreshToken>();
    public DbSet<Robot> Robots => Set<Robot>();
    public DbSet<Sensor> Sensors => Set<Sensor>();
    public DbSet<SensorReading> SensorReadings => Set<SensorReading>();
    public DbSet<TaskTemplate> TaskTemplates => Set<TaskTemplate>();
    public DbSet<TaskLog> TaskLogs => Set<TaskLog>();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="modelBuilder"></param>
    protected override void OnModelCreating(ModelBuilder modelBuilder)
    {
        modelBuilder.HasPostgresEnum<SensorTypeEnum>();
        modelBuilder.HasPostgresEnum<TaskTypeEnum>();

    }


}
