using System;
using Microsoft.EntityFrameworkCore.Migrations;
using Npgsql.EntityFrameworkCore.PostgreSQL.Metadata;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddTask : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AlterDatabase()
                .Annotation("Npgsql:Enum:sensor_type_enum", "none,speed,laser_distance,encoder,inercial,motor_speed_percentage")
                .Annotation("Npgsql:Enum:task_type_enum", "none,transport,follow_leader,formation")
                .OldAnnotation("Npgsql:Enum:sensor_type_enum", "none,speed,laser_distance,encoder,inercial,motor_speed_percentage");

            migrationBuilder.CreateTable(
                name: "TaskTemplates",
                columns: table => new
                {
                    Id = table.Column<int>(type: "integer", nullable: false)
                        .Annotation("Npgsql:ValueGenerationStrategy", NpgsqlValueGenerationStrategy.IdentityByDefaultColumn),
                    DateCreated = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                    TaskType = table.Column<int>(type: "integer", nullable: false),
                    Name = table.Column<string>(type: "text", nullable: false)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_TaskTemplates", x => x.Id);
                });

            migrationBuilder.CreateTable(
                name: "TaskLogs",
                columns: table => new
                {
                    Id = table.Column<int>(type: "integer", nullable: false)
                        .Annotation("Npgsql:ValueGenerationStrategy", NpgsqlValueGenerationStrategy.IdentityByDefaultColumn),
                    DateCreated = table.Column<DateTime>(type: "timestamp without time zone", nullable: false),
                    DateFinished = table.Column<DateTime>(type: "timestamp without time zone", nullable: true),
                    TaskTemplateId = table.Column<int>(type: "integer", nullable: false),
                    RobotId = table.Column<int>(type: "integer", nullable: false)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_TaskLogs", x => x.Id);
                    table.ForeignKey(
                        name: "FK_TaskLogs_Robots_RobotId",
                        column: x => x.RobotId,
                        principalTable: "Robots",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                    table.ForeignKey(
                        name: "FK_TaskLogs_TaskTemplates_TaskTemplateId",
                        column: x => x.TaskTemplateId,
                        principalTable: "TaskTemplates",
                        principalColumn: "Id",
                        onDelete: ReferentialAction.Cascade);
                });

            migrationBuilder.CreateIndex(
                name: "IX_TaskLogs_RobotId",
                table: "TaskLogs",
                column: "RobotId");

            migrationBuilder.CreateIndex(
                name: "IX_TaskLogs_TaskTemplateId",
                table: "TaskLogs",
                column: "TaskTemplateId");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "TaskLogs");

            migrationBuilder.DropTable(
                name: "TaskTemplates");

            migrationBuilder.AlterDatabase()
                .Annotation("Npgsql:Enum:sensor_type_enum", "none,speed,laser_distance,encoder,inercial,motor_speed_percentage")
                .OldAnnotation("Npgsql:Enum:sensor_type_enum", "none,speed,laser_distance,encoder,inercial,motor_speed_percentage")
                .OldAnnotation("Npgsql:Enum:task_type_enum", "none,transport,follow_leader,formation");
        }
    }
}
