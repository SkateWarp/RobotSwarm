using System;
using Microsoft.EntityFrameworkCore.Migrations;
using Npgsql.EntityFrameworkCore.PostgreSQL.Metadata;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddRobotGroup : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<int>(
                name: "RobotGroupId",
                table: "TaskLogs",
                type: "integer",
                nullable: true);

            migrationBuilder.AddColumn<int>(
                name: "RobotGroupId",
                table: "Robots",
                type: "integer",
                nullable: true);

            migrationBuilder.CreateTable(
                name: "RobotGroups",
                columns: table => new
                {
                    Id = table.Column<int>(type: "integer", nullable: false)
                        .Annotation("Npgsql:ValueGenerationStrategy", NpgsqlValueGenerationStrategy.IdentityByDefaultColumn),
                    Name = table.Column<string>(type: "text", nullable: false),
                    Description = table.Column<string>(type: "text", nullable: true),
                    DateCreated = table.Column<DateTime>(type: "timestamp without time zone", nullable: false)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_RobotGroups", x => x.Id);
                });

            migrationBuilder.CreateIndex(
                name: "IX_TaskLogs_RobotGroupId",
                table: "TaskLogs",
                column: "RobotGroupId");

            migrationBuilder.CreateIndex(
                name: "IX_Robots_RobotGroupId",
                table: "Robots",
                column: "RobotGroupId");

            migrationBuilder.AddForeignKey(
                name: "FK_Robots_RobotGroups_RobotGroupId",
                table: "Robots",
                column: "RobotGroupId",
                principalTable: "RobotGroups",
                principalColumn: "Id");

            migrationBuilder.AddForeignKey(
                name: "FK_TaskLogs_RobotGroups_RobotGroupId",
                table: "TaskLogs",
                column: "RobotGroupId",
                principalTable: "RobotGroups",
                principalColumn: "Id");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Robots_RobotGroups_RobotGroupId",
                table: "Robots");

            migrationBuilder.DropForeignKey(
                name: "FK_TaskLogs_RobotGroups_RobotGroupId",
                table: "TaskLogs");

            migrationBuilder.DropTable(
                name: "RobotGroups");

            migrationBuilder.DropIndex(
                name: "IX_TaskLogs_RobotGroupId",
                table: "TaskLogs");

            migrationBuilder.DropIndex(
                name: "IX_Robots_RobotGroupId",
                table: "Robots");

            migrationBuilder.DropColumn(
                name: "RobotGroupId",
                table: "TaskLogs");

            migrationBuilder.DropColumn(
                name: "RobotGroupId",
                table: "Robots");
        }
    }
}
