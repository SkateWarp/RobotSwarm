using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddRobotAccessControl : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<int>(
                name: "AccountId",
                table: "Robots",
                type: "integer",
                nullable: true);

            migrationBuilder.AddColumn<bool>(
                name: "IsPublic",
                table: "Robots",
                type: "boolean",
                nullable: false,
                defaultValue: false);

            migrationBuilder.CreateIndex(
                name: "IX_Robots_AccountId",
                table: "Robots",
                column: "AccountId");

            migrationBuilder.AddForeignKey(
                name: "FK_Robots_Accounts_AccountId",
                table: "Robots",
                column: "AccountId",
                principalTable: "Accounts",
                principalColumn: "Id");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropForeignKey(
                name: "FK_Robots_Accounts_AccountId",
                table: "Robots");

            migrationBuilder.DropIndex(
                name: "IX_Robots_AccountId",
                table: "Robots");

            migrationBuilder.DropColumn(
                name: "AccountId",
                table: "Robots");

            migrationBuilder.DropColumn(
                name: "IsPublic",
                table: "Robots");
        }
    }
}
