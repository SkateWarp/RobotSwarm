import { useState } from "react";

const useActualCompanyIdWithState = () => {
    const [companyId, setCompanyId] = useState(null);

    return [companyId, setCompanyId];
};

export default useActualCompanyIdWithState;
