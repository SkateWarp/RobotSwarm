using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class TaskCancelleDate : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<DateTime>(
                name: "DateCancelled",
                table: "TaskLogs",
                type: "timestamp without time zone",
                nullable: true);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "DateCancelled",
                table: "TaskLogs");
        }
    }
}
