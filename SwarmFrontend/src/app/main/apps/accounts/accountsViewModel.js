export const ACCOUNTS_PAGE_SIZE = 10;

export const normalizeAccount = (account) => ({
    ...account,
    // Las respuestas anteriores no incluían Enabled. En ese caso se conserva
    // el comportamiento histórico y la cuenta se considera activa.
    enabled: account.enabled !== false,
});

export const isCurrentAccount = (account, currentUser) => {
    if (account?.id !== undefined && currentUser?.id !== undefined) {
        return String(account.id) === String(currentUser.id);
    }

    return !!account?.email && account.email.toLocaleLowerCase() === currentUser?.email?.toLocaleLowerCase();
};

const includesText = (account, searchText) => {
    const value = [account.firstName, account.lastName, account.email]
        .filter(Boolean)
        .join(" ")
        .toLocaleLowerCase();

    return value.includes(searchText.trim().toLocaleLowerCase());
};

export const filterAccounts = (accounts, { role = "all", status = "active", searchText = "" }) => {
    return accounts.map(normalizeAccount).filter((account) => {
        if (status === "active" && !account.enabled) {
            return false;
        }
        if (status === "inactive" && account.enabled) {
            return false;
        }
        if (role !== "all" && account.role !== role) {
            return false;
        }
        return !searchText.trim() || includesText(account, searchText);
    });
};
