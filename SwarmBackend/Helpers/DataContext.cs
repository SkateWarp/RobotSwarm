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
    public DbSet<RobotGroup> RobotGroups => Set<RobotGroup>();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="modelBuilder"></param>
    protected override void OnModelCreating(ModelBuilder modelBuilder)
    {
        var dateCreated = new DateTime(2023, 4, 15);
        modelBuilder.HasPostgresEnum<SensorTypeEnum>();
        modelBuilder.HasPostgresEnum<TaskTypeEnum>();

        modelBuilder.Entity<TaskTemplate>()
            .HasData(new TaskTemplate[]
            {
                new TaskTemplate
                {
                    Id = 1,
                    DateCreated = dateCreated,
                    Name = "One",
                    TaskType = TaskTypeEnum.Transport
                },
                new TaskTemplate
                {
                    Id = 2,
                    DateCreated = dateCreated,
                    Name = "One",
                    TaskType = TaskTypeEnum.Transport
                },
                new TaskTemplate
                {
                    Id = 3,
                    DateCreated = dateCreated,
                    Name = "Two",
                    TaskType = TaskTypeEnum.FollowLeader
                },
                new TaskTemplate
                {
                    Id = 4,
                    DateCreated = dateCreated,
                    Name = "Third",
                    TaskType = TaskTypeEnum.Formation
                },
            });


    }


}
