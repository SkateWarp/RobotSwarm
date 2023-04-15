using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

#pragma warning disable CA1814 // Prefer jagged arrays over multidimensional

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddTaskTemplatesSeed : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.InsertData(
                table: "TaskTemplates",
                columns: new[] { "Id", "DateCreated", "Name", "TaskType" },
                values: new object[,]
                {
                    { 1, new DateTime(2023, 4, 15, 0, 0, 0, 0, DateTimeKind.Unspecified), "One", 1 },
                    { 2, new DateTime(2023, 4, 15, 0, 0, 0, 0, DateTimeKind.Unspecified), "One", 1 },
                    { 3, new DateTime(2023, 4, 15, 0, 0, 0, 0, DateTimeKind.Unspecified), "Two", 2 },
                    { 4, new DateTime(2023, 4, 15, 0, 0, 0, 0, DateTimeKind.Unspecified), "Third", 3 }
                });
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DeleteData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 1);

            migrationBuilder.DeleteData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 2);

            migrationBuilder.DeleteData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 3);

            migrationBuilder.DeleteData(
                table: "TaskTemplates",
                keyColumn: "Id",
                keyValue: 4);
        }
    }
}
