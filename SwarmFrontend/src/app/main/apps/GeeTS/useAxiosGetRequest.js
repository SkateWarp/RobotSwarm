/* eslint-disable react-hooks/exhaustive-deps */
import { useEffect, useState } from "react";
import axios from "axios";
import { URL } from "../../../constants/constants";

const useAxiosGetRequest = (completeUrl, initialState) => {
    const [data, setData] = useState(initialState);

    useEffect(() => {
        axios.get(`${URL}${completeUrl}`).then((response) => {
            setData(response.data);
        });
    }, []);

    return data;
};

export default useAxiosGetRequest;
