using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Migrations;
using SwarmBackend.Helpers;

#nullable disable

namespace SwarmBackend.Migrations;

[DbContext(typeof(DataContext))]
[Migration("20260716020000_AddSessionControls")]
public partial class AddSessionControls : Migration
{
    protected override void Up(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.AddColumn<Guid>(
            name: "TaskRunId",
            table: "WorkerCommands",
            type: "uuid",
            nullable: true);

        migrationBuilder.AddColumn<bool>(
            name: "IsEmergencyStopped",
            table: "SimulationSessions",
            type: "boolean",
            nullable: false,
            defaultValue: false);

        migrationBuilder.AddColumn<string>(
            name: "IdempotencyKey",
            table: "ViewerLeases",
            type: "character varying(200)",
            maxLength: 200,
            nullable: true);

        migrationBuilder.CreateIndex(
            name: "IX_WorkerCommands_TaskRunId",
            table: "WorkerCommands",
            column: "TaskRunId");

        migrationBuilder.CreateIndex(
            name: "IX_ViewerLeases_SimulationSessionId_IdempotencyKey",
            table: "ViewerLeases",
            columns: new[] { "SimulationSessionId", "IdempotencyKey" },
            unique: true);

        migrationBuilder.AddForeignKey(
            name: "FK_WorkerCommands_TaskRuns_TaskRunId",
            table: "WorkerCommands",
            column: "TaskRunId",
            principalTable: "TaskRuns",
            principalColumn: "Id",
            onDelete: ReferentialAction.SetNull);
    }

    protected override void Down(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.DropForeignKey(
            name: "FK_WorkerCommands_TaskRuns_TaskRunId",
            table: "WorkerCommands");
        migrationBuilder.DropIndex(
            name: "IX_WorkerCommands_TaskRunId",
            table: "WorkerCommands");
        migrationBuilder.DropIndex(
            name: "IX_ViewerLeases_SimulationSessionId_IdempotencyKey",
            table: "ViewerLeases");
        migrationBuilder.DropColumn(name: "TaskRunId", table: "WorkerCommands");
        migrationBuilder.DropColumn(name: "IsEmergencyStopped", table: "SimulationSessions");
        migrationBuilder.DropColumn(name: "IdempotencyKey", table: "ViewerLeases");
    }
}
