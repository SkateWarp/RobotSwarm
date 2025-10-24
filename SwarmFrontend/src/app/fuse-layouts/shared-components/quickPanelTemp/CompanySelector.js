import { FormControl, MenuItem, TextField } from "@mui/material";
import { useEffect, useState } from "react";
import { URL } from "app/constants/constants";
import axios from "axios";
import jwtService from "app/services/jwtService/jwtService";
import useActualCompanyIdWithState from "app/shared-components/hooks/useActualCompanyIdWithState";
import history from "@history";
import settingsConfig from "app/fuse-configs/settingsConfig";
import { useSelector } from "react-redux";

function CompanySelector() {
    const [companies, setCompanies] = useState([]);
    const user = useSelector(({ auth }) => auth?.user);
    const [companyId, setCompanyId] = useActualCompanyIdWithState();
    const canLoadCompanySelector =
        user && (user.id === "1" || user.id === 1) && settingsConfig.layout.project === "panelTemp";
    useEffect(() => {
        if (companies.length === 0 && canLoadCompanySelector)
            axios
                .get(`${URL}/api/Company`, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${jwtService.getAccessToken()}`,
                    },
                })
                .then((res) => {
                    setCompanies(res.data);
                });
    }, [companies.length, canLoadCompanySelector]);

    if (!canLoadCompanySelector) return null;

    return (
        <FormControl className="w-128" variant="outlined">
            <TextField
                sx={{ height: "5rem" }}
                select
                label="Empresa"
                value={companyId}
                onChange={(event) => {
                    setCompanyId(event.target.value);
                }}
            >
                {companies.map((company) => (
                    <MenuItem value={company.id} key={company.id}>
                        {company.name}
                    </MenuItem>
                ))}
            </TextField>
        </FormControl>
    );
}

export default CompanySelector;
