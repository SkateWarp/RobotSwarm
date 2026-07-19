import { ACCOUNTS_PAGE_SIZE, filterAccounts, isCurrentAccount, normalizeAccount } from "./accountsViewModel";

const accounts = [
    {
        id: 1,
        firstName: "Ada",
        lastName: "Lovelace",
        email: "ada@example.test",
        role: "Admin",
        enabled: true,
    },
    {
        id: 2,
        firstName: "Grace",
        lastName: "Hopper",
        email: "grace@example.test",
        role: "User",
        enabled: false,
    },
    {
        id: 3,
        firstName: "Alan",
        lastName: "Turing",
        email: "alan@example.test",
        role: "User",
        enabled: true,
    },
];

describe("accountsViewModel", () => {
    it("uses ten rows as the default client-side page", () => {
        expect(ACCOUNTS_PAGE_SIZE).toBe(10);
    });

    it("keeps older account responses visible by treating a missing enabled field as active", () => {
        expect(normalizeAccount({ id: 9, role: "User" }).enabled).toBe(true);
    });

    it("shows active accounts by default", () => {
        expect(filterAccounts(accounts, {}).map((account) => account.id)).toEqual([1, 3]);
    });

    it("combines status, role and local text filters", () => {
        expect(
            filterAccounts(accounts, {
                status: "inactive",
                role: "User",
                searchText: "HOPPER",
            }).map((account) => account.id)
        ).toEqual([2]);
    });

    it("identifies the current account by id, with email as a fallback", () => {
        expect(isCurrentAccount({ id: 7 }, { id: "7" })).toBe(true);
        expect(isCurrentAccount({ email: "admin@example.test" }, { email: "ADMIN@example.test" })).toBe(true);
    });
});
