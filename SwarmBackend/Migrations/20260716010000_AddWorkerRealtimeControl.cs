using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Migrations;
using SwarmBackend.Helpers;

#nullable disable

namespace SwarmBackend.Migrations;

[DbContext(typeof(DataContext))]
[Migration("20260716010000_AddWorkerRealtimeControl")]
public partial class AddWorkerRealtimeControl : Migration
{
    protected override void Up(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.AddColumn<System.Text.Json.JsonDocument>(
            name: "Result",
            table: "WorkerCommands",
            type: "jsonb",
            nullable: true);

        migrationBuilder.AddColumn<DateTime>(
            name: "CredentialCreatedAt",
            table: "ComputeWorkers",
            type: "timestamp without time zone",
            nullable: true);

        migrationBuilder.AddColumn<string>(
            name: "CredentialHash",
            table: "ComputeWorkers",
            type: "character varying(64)",
            maxLength: 64,
            nullable: true);

        migrationBuilder.AddColumn<DateTime>(
            name: "CredentialRevokedAt",
            table: "ComputeWorkers",
            type: "timestamp without time zone",
            nullable: true);

        migrationBuilder.CreateIndex(
            name: "IX_ComputeWorkers_State_LastHeartbeatAt",
            table: "ComputeWorkers",
            columns: new[] { "State", "LastHeartbeatAt" });
    }

    protected override void Down(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.DropIndex(
            name: "IX_ComputeWorkers_State_LastHeartbeatAt",
            table: "ComputeWorkers");

        migrationBuilder.DropColumn(name: "Result", table: "WorkerCommands");
        migrationBuilder.DropColumn(name: "CredentialCreatedAt", table: "ComputeWorkers");
        migrationBuilder.DropColumn(name: "CredentialHash", table: "ComputeWorkers");
        migrationBuilder.DropColumn(name: "CredentialRevokedAt", table: "ComputeWorkers");
    }
}
