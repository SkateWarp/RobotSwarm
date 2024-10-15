using System.Text.Json;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class TaskParameters : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<JsonDocument>(
                name: "Parameters",
                table: "TaskLogs",
                type: "jsonb",
                nullable: false,
                defaultValue: JsonDocument.Parse("{}"));
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Parameters",
                table: "TaskLogs");
        }
    }
}
