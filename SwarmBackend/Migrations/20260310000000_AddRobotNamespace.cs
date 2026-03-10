using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddRobotNamespace : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<string>(
                name: "Namespace",
                table: "Robots",
                type: "text",
                nullable: true);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Namespace",
                table: "Robots");
        }
    }
}
