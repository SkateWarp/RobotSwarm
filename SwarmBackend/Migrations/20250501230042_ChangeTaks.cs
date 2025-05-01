using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class ChangeTaks : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_TaskLogs_Robots_RobotId",
                table: "TaskLogs");

            migrationBuilder.DropIndex(
                name: "IX_TaskLogs_RobotId",
                table: "TaskLogs");

            migrationBuilder.DropColumn(
                name: "RobotId",
                table: "TaskLogs");

            migrationBuilder.AddColumn<bool>(
                name: "Enabled",
                table: "TaskTemplates",
                type: "boolean",
                nullable: false,
                defaultValue: false);

            migrationBuilder.AddColumn<int>(
                name: "TaskLogId",
                table: "Robots",
                type: "integer",
                nullable: true);

            migrationBuilder.UpdateData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 1,
                column: "Enabled",
                value: true);

            migrationBuilder.UpdateData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 2,
                column: "Enabled",
                value: true);

            migrationBuilder.UpdateData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 3,
                column: "Enabled",
                value: true);

            migrationBuilder.UpdateData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 4,
                column: "Enabled",
                value: true);

            migrationBuilder.CreateIndex(
                name: "IX_Robots_TaskLogId",
                table: "Robots",
                column: "TaskLogId");

            migrationBuilder.AddForeignKey(
                name: "FK_Robots_TaskLogs_TaskLogId",
                table: "Robots",
                column: "TaskLogId",
                principalTable: "TaskLogs",
                principalColumn: "Id");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Robots_TaskLogs_TaskLogId",
                table: "Robots");

            migrationBuilder.DropIndex(
                name: "IX_Robots_TaskLogId",
                table: "Robots");

            migrationBuilder.DropColumn(
                name: "Enabled",
                table: "TaskTemplates");

            migrationBuilder.DropColumn(
                name: "TaskLogId",
                table: "Robots");

            migrationBuilder.AddColumn<int>(
                name: "RobotId",
                table: "TaskLogs",
                type: "integer",
                nullable: false,
                defaultValue: 0);

            migrationBuilder.CreateIndex(
                name: "IX_TaskLogs_RobotId",
                table: "TaskLogs",
                column: "RobotId");

            migrationBuilder.AddForeignKey(
                name: "FK_TaskLogs_Robots_RobotId",
                table: "TaskLogs",
                column: "RobotId",
                principalTable: "Robots",
                principalColumn: "Id",
                onDelete: ReferentialAction.Cascade);
        }
    }
}
