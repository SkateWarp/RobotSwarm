using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace SwarmBackend.Migrations
{
    /// <inheritdoc />
    public partial class AddCanonicalAccountEmail : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<string>(
                name: "NormalizedEmail",
                table: "Accounts",
                type: "text",
                nullable: false,
                computedColumnSql: "lower(btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)))",
                stored: true);

            // Do not guess which legacy account should keep an address. EF runs
            // migrations in a transaction, so this diagnostic aborts the whole
            // migration and leaves the old schema and data untouched.
            migrationBuilder.Sql(
                """
                DO $$
                BEGIN
                    IF EXISTS (
                        SELECT 1
                        FROM "Accounts"
                        WHERE char_length(btrim(
                                "Email",
                                ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)
                            )) NOT BETWEEN 1 AND 254
                           OR btrim(
                                "Email",
                                ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)
                            ) !~ '^[A-Za-z0-9._%+-]+@([A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)+[A-Za-z]{2,63}$'
                    ) THEN
                        RAISE EXCEPTION USING
                            ERRCODE = '23514',
                            MESSAGE = 'Unsupported legacy account emails prevent the canonical-email migration.',
                            HINT = 'Review account emails for length and the supported ASCII form, then resolve them manually before retrying.';
                    END IF;
                END
                $$;
                """);

            migrationBuilder.AddCheckConstraint(
                name: "CK_Accounts_Email_SupportedFormat",
                table: "Accounts",
                sql: "char_length(btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13))) BETWEEN 1 AND 254 "
                    + "AND btrim(\"Email\", ' ' || chr(9) || chr(10) || chr(11) || chr(12) || chr(13)) "
                    + "~ '^[A-Za-z0-9._%+-]+@([A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?\\.)+[A-Za-z]{2,63}$'");

            migrationBuilder.Sql(
                """
                DO $$
                BEGIN
                    IF EXISTS (
                        SELECT 1
                        FROM "Accounts"
                        GROUP BY "NormalizedEmail"
                        HAVING COUNT(*) > 1
                    ) THEN
                        RAISE EXCEPTION USING
                            ERRCODE = '23505',
                            MESSAGE = 'Canonical duplicate account emails prevent the uniqueness migration.',
                            HINT = 'Review accounts grouped by the generated NormalizedEmail and resolve each duplicate manually before retrying.';
                    END IF;
                END
                $$;
                """);

            migrationBuilder.CreateIndex(
                name: "IX_Accounts_NormalizedEmail",
                table: "Accounts",
                column: "NormalizedEmail",
                unique: true);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropIndex(
                name: "IX_Accounts_NormalizedEmail",
                table: "Accounts");

            migrationBuilder.DropCheckConstraint(
                name: "CK_Accounts_Email_SupportedFormat",
                table: "Accounts");

            migrationBuilder.DropColumn(
                name: "NormalizedEmail",
                table: "Accounts");
        }
    }
}
