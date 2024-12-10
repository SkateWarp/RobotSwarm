/* eslint-disable react-hooks/exhaustive-deps */
import {useEffect, useState} from "react";
import axios from "axios";
import {URL} from "../../../constants/constants";
import jwtService from "../../../services/jwtService";

const useAxiosGetRequest = (completeUrl, initialState) => {

    const [data, setData] = useState(initialState);

    useEffect(() => {
        axios.get(`${URL}${completeUrl}`, {
            headers: {
                "Content-Type": "application/json",
                Authorization: `Bearer ${jwtService.getAccessToken()}`,
            },
        }).then((response) => {
            setData(response.data);
        });
    }, []);

    return data;
};

export default useAxiosGetRequest;
