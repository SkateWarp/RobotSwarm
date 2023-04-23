const useFragaStopTypeText = () => {
    return (status) => {
        switch (status) {
            case 0:
                return "PARADA OPERACIONAL";
            case 1:
                return "FALLA TECNICA";
            case 2:
                return "PARADA PROGRAMADA";
            case 3:
                return "EXTERNA";
            case 4:
                return "OTROS";
            case 5:
                return "MAQUINA";
            case 6:
                return "PRODUCCION";
            default:
                return "";
        }
    };
};

export default useFragaStopTypeText;
