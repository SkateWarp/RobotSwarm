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
                return "Nigga";
           
                

            default:
                return "";
        }
    };

    return getText(status);
};

export default useTaskCategoriesText;
