using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddWorkerDrainAndTaskOutcomes : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<DateTime>(
                name: "LastProgressAt",
                table: "TaskRuns",
                type: "timestamp without time zone",
                nullable: true);

            migrationBuilder.AddColumn<DateTime>(
                name: "LastReportAt",
                table: "TaskRuns",
                type: "timestamp without time zone",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "OutcomeReason",
                table: "TaskRuns",
                type: "character varying(4000)",
                maxLength: 4000,
                nullable: true);

            migrationBuilder.AddColumn<int>(
                name: "OutcomeState",
                table: "TaskRuns",
                type: "integer",
                nullable: false,
                defaultValue: 0);

            migrationBuilder.AddColumn<DateTime>(
                name: "ActiveSessionsReportedAt",
                table: "ComputeWorkers",
                type: "timestamp without time zone",
                nullable: true);

            migrationBuilder.AddColumn<DateTime>(
                name: "DrainLeaseExpiresAt",
                table: "ComputeWorkers",
                type: "timestamp without time zone",
                nullable: true);

            migrationBuilder.AddColumn<Guid>(
                name: "DrainLeaseId",
                table: "ComputeWorkers",
                type: "uuid",
                nullable: true);

            migrationBuilder.AddColumn<DateTime>(
                name: "DrainRequestedAt",
                table: "ComputeWorkers",
                type: "timestamp without time zone",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "DrainTargetRevision",
                table: "ComputeWorkers",
                type: "character varying(40)",
                maxLength: 40,
                nullable: true);

            migrationBuilder.AddColumn<int>(
                name: "ReportedActiveSessionCount",
                table: "ComputeWorkers",
                type: "integer",
                nullable: true);

            migrationBuilder.Sql(
                """
                UPDATE "TaskRuns"
                SET "OutcomeState" = CASE "State"
                    WHEN 5 THEN 1
                    WHEN 6 THEN 3
                    WHEN 7 THEN 2
                    ELSE 0
                END,
                "OutcomeReason" = CASE WHEN "State" = 7 THEN "Error" ELSE NULL END,
                "LastReportAt" = CASE
                    WHEN "State" IN (1, 2) THEN "UpdatedAt"
                    ELSE NULL
                END,
                "LastProgressAt" = CASE
                    WHEN "State" IN (1, 2) THEN "UpdatedAt"
                    ELSE NULL
                END;
                """);

            migrationBuilder.CreateIndex(
                name: "IX_TaskRuns_State_LastProgressAt",
                table: "TaskRuns",
                columns: new[] { "State", "LastProgressAt" });

            migrationBuilder.CreateIndex(
                name: "IX_ComputeWorkers_DrainLeaseId",
                table: "ComputeWorkers",
                column: "DrainLeaseId",
                unique: true);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropIndex(
                name: "IX_TaskRuns_State_LastProgressAt",
                table: "TaskRuns");

            migrationBuilder.DropIndex(
                name: "IX_ComputeWorkers_DrainLeaseId",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "LastProgressAt",
                table: "TaskRuns");

            migrationBuilder.DropColumn(
                name: "LastReportAt",
                table: "TaskRuns");

            migrationBuilder.DropColumn(
                name: "OutcomeReason",
                table: "TaskRuns");

            migrationBuilder.DropColumn(
                name: "OutcomeState",
                table: "TaskRuns");

            migrationBuilder.DropColumn(
                name: "ActiveSessionsReportedAt",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "DrainLeaseExpiresAt",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "DrainLeaseId",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "DrainRequestedAt",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "DrainTargetRevision",
                table: "ComputeWorkers");

            migrationBuilder.DropColumn(
                name: "ReportedActiveSessionCount",
                table: "ComputeWorkers");
        }
    }
}
