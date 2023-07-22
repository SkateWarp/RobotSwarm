const useTaskCategoriesText = (status) => {
    const getText = (option) => {
        switch (option) {
            case 0:
                return "Ninguno";
            case 1:
                return "Transporte";
            case 2:
                return "Sigue al líder";
            case 3:
                return "Formación";
            default:
                return "";
        }
    };

    return getText(status);
};

export default useTaskCategoriesText;
