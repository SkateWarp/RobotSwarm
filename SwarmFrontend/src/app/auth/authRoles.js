/**
 * Authorization Roles
 */
const authRoles = {
    admin: ["Admin"],
    reception: ["Admin", "Reception"],
    user: ["Admin", "User"],
    // all: ['Admin', 'User', 'Operator', 'Maintenance', 'CleanOperator', 'WasteOperator', 'Reception'],
    clean: ["Admin", "User", "Operator", "Maintenance", "CleanOperator"],
    waste: ["Admin", "User", "Operator", "Maintenance", "WasteOperator"],
    maintenance: ["Admin", "Maintenance", "User", "Supervisor"],
    supervisor: ["Admin", "Supervisor", "Maintenance"],
    //Solo puede entrar a creacion de ordenes de tola y aluzinc
    sales: ["Admin", "Sales", "User"],
    sales_maintenance: ["Admin", "Sales", "User", "Maintenance"],
    onlyGuest: [],
};

export default authRoles;
