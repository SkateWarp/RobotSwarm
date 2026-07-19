using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Migrations;
using SwarmBackend.Helpers;

#nullable disable

namespace SwarmBackend.Migrations;

[DbContext(typeof(DataContext))]
[Migration("20260717000000_AddViewerPublishToken")]
public partial class AddViewerPublishToken : Migration
{
    protected override void Up(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.AddColumn<string>(
            name: "PublishTokenHash",
            table: "ViewerLeases",
            type: "character varying(64)",
            maxLength: 64,
            nullable: true);

        migrationBuilder.CreateIndex(
            name: "IX_ViewerLeases_PublishTokenHash",
            table: "ViewerLeases",
            column: "PublishTokenHash",
            unique: true);
    }

    protected override void Down(MigrationBuilder migrationBuilder)
    {
        migrationBuilder.DropIndex(
            name: "IX_ViewerLeases_PublishTokenHash",
            table: "ViewerLeases");

        migrationBuilder.DropColumn(
            name: "PublishTokenHash",
            table: "ViewerLeases");
    }
}
