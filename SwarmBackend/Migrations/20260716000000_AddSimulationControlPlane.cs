using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Migrations;
using SwarmBackend.Helpers;

#nullable disable

namespace SwarmBackend.Migrations;

[DbContext(typeof(DataContext))]
[Migration("20260716000000_AddSimulationControlPlane")]
public partial class AddSimulationControlPlane : Migration
{
    protected override void Up(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.CreateTable(
            name: "ComputeWorkers",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                Name = table.Column<string>(type: "character varying(100)", maxLength: 100, nullable: false),
                State = table.Column<int>(type: "integer", nullable: false),
                MaxConcurrentSessions = table.Column<int>(type: "integer", nullable: false),
                ImageVersion = table.Column<string>(type: "character varying(200)", maxLength: 200, nullable: true),
                Capabilities = table.Column<System.Text.Json.JsonDocument>(type: "jsonb", nullable: false),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                UpdatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                LastHeartbeatAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_ComputeWorkers", x => x.Id);
            });

        migrationBuilder.CreateTable(
            name: "SimulationSessions",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                AccountId = table.Column<int>(type: "integer", nullable: false),
                ComputeWorkerId = table.Column<Guid>(type: "uuid", nullable: true),
                State = table.Column<int>(type: "integer", nullable: false),
                DesiredRobotCount = table.Column<int>(type: "integer", nullable: false),
                ArenaVersion = table.Column<string>(type: "character varying(100)", maxLength: 100, nullable: false),
                WorkerImageVersion = table.Column<string>(type: "character varying(200)", maxLength: 200, nullable: true),
                Revision = table.Column<long>(type: "bigint", nullable: false),
                FailureReason = table.Column<string>(type: "character varying(2000)", maxLength: 2000, nullable: true),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                UpdatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                StartedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true),
                StoppedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_SimulationSessions", x => x.Id);
                table.CheckConstraint(
                    "CK_SimulationSessions_DesiredRobotCount_Positive",
                    "\"DesiredRobotCount\" > 0");
                table.ForeignKey(
                    name: "FK_SimulationSessions_Accounts_AccountId",
                    column: x => x.AccountId,
                    principalTable: "Accounts",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Restrict);
                table.ForeignKey(
                    name: "FK_SimulationSessions_ComputeWorkers_ComputeWorkerId",
                    column: x => x.ComputeWorkerId,
                    principalTable: "ComputeWorkers",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.SetNull);
            });

        migrationBuilder.CreateTable(
            name: "SessionRobots",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                SimulationSessionId = table.Column<Guid>(type: "uuid", nullable: false),
                Ordinal = table.Column<int>(type: "integer", nullable: false),
                RuntimeId = table.Column<string>(type: "character varying(100)", maxLength: 100, nullable: false),
                Namespace = table.Column<string>(type: "character varying(200)", maxLength: 200, nullable: false),
                Role = table.Column<string>(type: "character varying(100)", maxLength: 100, nullable: true),
                State = table.Column<int>(type: "integer", nullable: false),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                UpdatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_SessionRobots", x => x.Id);
                table.ForeignKey(
                    name: "FK_SessionRobots_SimulationSessions_SimulationSessionId",
                    column: x => x.SimulationSessionId,
                    principalTable: "SimulationSessions",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Cascade);
            });

        migrationBuilder.CreateTable(
            name: "TaskRuns",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                SimulationSessionId = table.Column<Guid>(type: "uuid", nullable: false),
                Type = table.Column<int>(type: "integer", nullable: false),
                State = table.Column<int>(type: "integer", nullable: false),
                Progress = table.Column<double>(type: "double precision", nullable: false),
                CommandRevision = table.Column<long>(type: "bigint", nullable: false),
                Parameters = table.Column<System.Text.Json.JsonDocument>(type: "jsonb", nullable: false),
                Result = table.Column<System.Text.Json.JsonDocument>(type: "jsonb", nullable: true),
                Error = table.Column<string>(type: "character varying(4000)", maxLength: 4000, nullable: true),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                UpdatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                StartedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true),
                CompletedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_TaskRuns", x => x.Id);
                table.CheckConstraint(
                    "CK_TaskRuns_Progress_Range",
                    "\"Progress\" >= 0 AND \"Progress\" <= 1");
                table.ForeignKey(
                    name: "FK_TaskRuns_SimulationSessions_SimulationSessionId",
                    column: x => x.SimulationSessionId,
                    principalTable: "SimulationSessions",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Cascade);
            });

        migrationBuilder.CreateTable(
            name: "ViewerLeases",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                SimulationSessionId = table.Column<Guid>(type: "uuid", nullable: false),
                AccountId = table.Column<int>(type: "integer", nullable: false),
                Source = table.Column<int>(type: "integer", nullable: false),
                RobotRuntimeId = table.Column<string>(type: "character varying(100)", maxLength: 100, nullable: true),
                TokenHash = table.Column<string>(type: "character varying(128)", maxLength: 128, nullable: false),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                ExpiresAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                RevokedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_ViewerLeases", x => x.Id);
                table.ForeignKey(
                    name: "FK_ViewerLeases_Accounts_AccountId",
                    column: x => x.AccountId,
                    principalTable: "Accounts",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Restrict);
                table.ForeignKey(
                    name: "FK_ViewerLeases_SimulationSessions_SimulationSessionId",
                    column: x => x.SimulationSessionId,
                    principalTable: "SimulationSessions",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Cascade);
            });

        migrationBuilder.CreateTable(
            name: "WorkerCommands",
            columns: table => new
            {
                Id = table.Column<Guid>(type: "uuid", nullable: false),
                SimulationSessionId = table.Column<Guid>(type: "uuid", nullable: false),
                ComputeWorkerId = table.Column<Guid>(type: "uuid", nullable: true),
                Type = table.Column<int>(type: "integer", nullable: false),
                State = table.Column<int>(type: "integer", nullable: false),
                IdempotencyKey = table.Column<string>(type: "character varying(200)", maxLength: 200, nullable: false),
                CorrelationId = table.Column<Guid>(type: "uuid", nullable: false),
                Sequence = table.Column<long>(type: "bigint", nullable: false),
                Payload = table.Column<System.Text.Json.JsonDocument>(type: "jsonb", nullable: false),
                RetryCount = table.Column<int>(type: "integer", nullable: false),
                LastError = table.Column<string>(type: "character varying(4000)", maxLength: 4000, nullable: true),
                CreatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                UpdatedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                DispatchedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true),
                AcknowledgedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true),
                CompletedAt = table.Column<DateTime>(type: "timestamp without time zone", nullable: true)
            },
            constraints: table =>
            {
                table.PrimaryKey("PK_WorkerCommands", x => x.Id);
                table.ForeignKey(
                    name: "FK_WorkerCommands_ComputeWorkers_ComputeWorkerId",
                    column: x => x.ComputeWorkerId,
                    principalTable: "ComputeWorkers",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.SetNull);
                table.ForeignKey(
                    name: "FK_WorkerCommands_SimulationSessions_SimulationSessionId",
                    column: x => x.SimulationSessionId,
                    principalTable: "SimulationSessions",
                    principalColumn: "Id",
                    onDelete: ReferentialAction.Cascade);
            });

        migrationBuilder.CreateIndex(
            name: "IX_ComputeWorkers_Name",
            table: "ComputeWorkers",
            column: "Name",
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_SessionRobots_SimulationSessionId_Namespace",
            table: "SessionRobots",
            columns: new[] { "SimulationSessionId", "Namespace" },
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_SessionRobots_SimulationSessionId_Ordinal",
            table: "SessionRobots",
            columns: new[] { "SimulationSessionId", "Ordinal" },
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_SessionRobots_SimulationSessionId_RuntimeId",
            table: "SessionRobots",
            columns: new[] { "SimulationSessionId", "RuntimeId" },
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_SimulationSessions_AccountId",
            table: "SimulationSessions",
            column: "AccountId",
            unique: true,
            filter: "\"State\" < 6");

        migrationBuilder.CreateIndex(
            name: "IX_SimulationSessions_ComputeWorkerId",
            table: "SimulationSessions",
            column: "ComputeWorkerId");

        migrationBuilder.CreateIndex(
            name: "IX_SimulationSessions_State_CreatedAt_Id",
            table: "SimulationSessions",
            columns: new[] { "State", "CreatedAt", "Id" });

        migrationBuilder.CreateIndex(
            name: "IX_TaskRuns_SimulationSessionId_CreatedAt",
            table: "TaskRuns",
            columns: new[] { "SimulationSessionId", "CreatedAt" });

        migrationBuilder.CreateIndex(
            name: "IX_ViewerLeases_AccountId",
            table: "ViewerLeases",
            column: "AccountId");

        migrationBuilder.CreateIndex(
            name: "IX_ViewerLeases_SimulationSessionId_ExpiresAt",
            table: "ViewerLeases",
            columns: new[] { "SimulationSessionId", "ExpiresAt" });

        migrationBuilder.CreateIndex(
            name: "IX_ViewerLeases_TokenHash",
            table: "ViewerLeases",
            column: "TokenHash",
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_WorkerCommands_ComputeWorkerId",
            table: "WorkerCommands",
            column: "ComputeWorkerId");

        migrationBuilder.CreateIndex(
            name: "IX_WorkerCommands_SimulationSessionId_IdempotencyKey",
            table: "WorkerCommands",
            columns: new[] { "SimulationSessionId", "IdempotencyKey" },
            unique: true);

        migrationBuilder.CreateIndex(
            name: "IX_WorkerCommands_SimulationSessionId_Sequence",
            table: "WorkerCommands",
            columns: new[] { "SimulationSessionId", "Sequence" },
            unique: true);
    }

    protected override void Down(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.DropTable(name: "SessionRobots");
        migrationBuilder.DropTable(name: "TaskRuns");
        migrationBuilder.DropTable(name: "ViewerLeases");
        migrationBuilder.DropTable(name: "WorkerCommands");
        migrationBuilder.DropTable(name: "SimulationSessions");
        migrationBuilder.DropTable(name: "ComputeWorkers");
    }
}
