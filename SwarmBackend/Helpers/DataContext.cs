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
    public DbSet<ComputeWorker> ComputeWorkers => Set<ComputeWorker>();
    public DbSet<SimulationSession> SimulationSessions => Set<SimulationSession>();
    public DbSet<SessionRobot> SessionRobots => Set<SessionRobot>();
    public DbSet<TaskRun> TaskRuns => Set<TaskRun>();
    public DbSet<WorkerCommand> WorkerCommands => Set<WorkerCommand>();
    public DbSet<ViewerLease> ViewerLeases => Set<ViewerLease>();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="modelBuilder"></param>
    protected override void OnModelCreating(ModelBuilder modelBuilder)
    {
        base.OnModelCreating(modelBuilder);

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

        modelBuilder.Entity<ComputeWorker>(entity =>
        {
            entity.Property(worker => worker.Name).HasMaxLength(100);
            entity.Property(worker => worker.ImageVersion).HasMaxLength(200);
            entity.Property(worker => worker.CredentialHash).HasMaxLength(64);
            entity.Property(worker => worker.Capabilities).HasColumnType("jsonb");
            entity.HasIndex(worker => worker.Name).IsUnique();
            entity.HasIndex(worker => new { worker.State, worker.LastHeartbeatAt });
        });

        modelBuilder.Entity<SimulationSession>(entity =>
        {
            entity.Property(session => session.ArenaVersion).HasMaxLength(100);
            entity.Property(session => session.WorkerImageVersion).HasMaxLength(200);
            entity.Property(session => session.FailureReason).HasMaxLength(2000);
            entity.ToTable(table => table.HasCheckConstraint(
                "CK_SimulationSessions_DesiredRobotCount_Positive",
                "\"DesiredRobotCount\" > 0"));
            entity.HasIndex(session => session.AccountId)
                .IsUnique()
                .HasFilter("\"State\" < 6");
            entity.HasIndex(session => new { session.State, session.CreatedAt, session.Id });
            entity.HasOne(session => session.Account)
                .WithMany(account => account.SimulationSessions)
                .HasForeignKey(session => session.AccountId)
                .OnDelete(DeleteBehavior.Restrict);
            entity.HasOne(session => session.ComputeWorker)
                .WithMany(worker => worker.Sessions)
                .HasForeignKey(session => session.ComputeWorkerId)
                .OnDelete(DeleteBehavior.SetNull);
        });

        modelBuilder.Entity<SessionRobot>(entity =>
        {
            entity.Property(robot => robot.RuntimeId).HasMaxLength(100);
            entity.Property(robot => robot.Namespace).HasMaxLength(200);
            entity.Property(robot => robot.Role).HasMaxLength(100);
            entity.HasIndex(robot => new { robot.SimulationSessionId, robot.Ordinal }).IsUnique();
            entity.HasIndex(robot => new { robot.SimulationSessionId, robot.RuntimeId }).IsUnique();
            entity.HasIndex(robot => new { robot.SimulationSessionId, robot.Namespace }).IsUnique();
            entity.HasOne(robot => robot.SimulationSession)
                .WithMany(session => session.Robots)
                .HasForeignKey(robot => robot.SimulationSessionId)
                .OnDelete(DeleteBehavior.Cascade);
        });

        modelBuilder.Entity<TaskRun>(entity =>
        {
            entity.Property(task => task.Parameters).HasColumnType("jsonb");
            entity.Property(task => task.Result).HasColumnType("jsonb");
            entity.Property(task => task.Error).HasMaxLength(4000);
            entity.ToTable(table => table.HasCheckConstraint(
                "CK_TaskRuns_Progress_Range",
                "\"Progress\" >= 0 AND \"Progress\" <= 1"));
            entity.HasIndex(task => new { task.SimulationSessionId, task.CreatedAt });
            entity.HasOne(task => task.SimulationSession)
                .WithMany(session => session.TaskRuns)
                .HasForeignKey(task => task.SimulationSessionId)
                .OnDelete(DeleteBehavior.Cascade);
        });

        modelBuilder.Entity<WorkerCommand>(entity =>
        {
            entity.Property(command => command.IdempotencyKey).HasMaxLength(200);
            entity.Property(command => command.Payload).HasColumnType("jsonb");
            entity.Property(command => command.Result).HasColumnType("jsonb");
            entity.Property(command => command.LastError).HasMaxLength(4000);
            entity.HasIndex(command => new { command.SimulationSessionId, command.IdempotencyKey })
                .IsUnique();
            entity.HasIndex(command => new { command.SimulationSessionId, command.Sequence })
                .IsUnique();
            entity.HasOne(command => command.SimulationSession)
                .WithMany(session => session.Commands)
                .HasForeignKey(command => command.SimulationSessionId)
                .OnDelete(DeleteBehavior.Cascade);
            entity.HasOne(command => command.TaskRun)
                .WithMany(task => task.Commands)
                .HasForeignKey(command => command.TaskRunId)
                .OnDelete(DeleteBehavior.SetNull);
            entity.HasOne(command => command.ComputeWorker)
                .WithMany(worker => worker.Commands)
                .HasForeignKey(command => command.ComputeWorkerId)
                .OnDelete(DeleteBehavior.SetNull);
        });

        modelBuilder.Entity<ViewerLease>(entity =>
        {
            entity.Property(lease => lease.RobotRuntimeId).HasMaxLength(100);
            entity.Property(lease => lease.IdempotencyKey).HasMaxLength(200);
            entity.Property(lease => lease.TokenHash).HasMaxLength(128);
            entity.Property(lease => lease.PublishTokenHash).HasMaxLength(64);
            entity.HasIndex(lease => lease.TokenHash).IsUnique();
            entity.HasIndex(lease => lease.PublishTokenHash).IsUnique();
            entity.HasIndex(lease => new { lease.SimulationSessionId, lease.ExpiresAt });
            entity.HasIndex(lease => new { lease.SimulationSessionId, lease.IdempotencyKey })
                .IsUnique();
            entity.HasOne(lease => lease.SimulationSession)
                .WithMany(session => session.ViewerLeases)
                .HasForeignKey(lease => lease.SimulationSessionId)
                .OnDelete(DeleteBehavior.Cascade);
            entity.HasOne(lease => lease.Account)
                .WithMany(account => account.ViewerLeases)
                .HasForeignKey(lease => lease.AccountId)
                .OnDelete(DeleteBehavior.Restrict);
        });

    }


}
