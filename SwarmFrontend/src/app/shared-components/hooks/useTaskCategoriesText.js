const useTaskCategoriesText = (status) => {
    const getText = (status) => {
        switch (status) {
            case 1:
                return "Transporte";
            case 2:
                return "Sigue al líder";
            case 3:
                return "Formación";
            case 4:
                return "Nigga";
           
                

            default:
                return "";
        }
    };

    return getText(status);
};

export default useTaskCategoriesText;
