const useTaskCategoriesText = (status) => {
    const getText = (status) => {
        switch (status) {
            case 1:
                return "Ventas";
            case 2:
                return "Gestión Humana";
            case 3:
                return "Contabilidad";
            case 4:
                return "Almacén";
            case 5:
                return "Operaciones";
            case 6:
                return "Proyectos";
            case 7:
                return "Tecnología";
            case 8:
                return "Mensajería";
            case 9:
                return "Compras";
            case 10:
                return "Mantenimiento";
            case 16:
                return "Gerencia";

            default:
                return "";
        }
    };

    return getText(status);
};

export default useTaskCategoriesText;
