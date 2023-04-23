import useActualCompanyIdWithState from "./useActualCompanyIdWithState";

const useActualCompanyId = () => {
    const [companyId] = useActualCompanyIdWithState();

    return companyId;
};

export default useActualCompanyId;
