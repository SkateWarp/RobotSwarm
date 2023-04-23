/* eslint-disable react-hooks/exhaustive-deps */
import { useEffect, useState } from "react";
import FuseUtils from "../../../@fuse/utils";

// custom hook general para componentes que no utilizan paginación en su componente list
const useFilteredData = (unfilteredDataList, searchText) => {
    const [statusFilter, setStatusFilter] = useState(false);
    const [filteredData, setFilteredData] = useState(null);

    useEffect(() => {
        const getFilteredArray = (entities, searchText) => {
            if (searchText.length === 0) return entities;

            return FuseUtils.filterArrayByString(entities, searchText);
        };

        const getFilteredData = () => {
            if (unfilteredDataList) {
                setFilteredData(getFilteredArray(unfilteredDataList, searchText));
            }

            if (unfilteredDataList.length) {
                setStatusFilter(true);
            } else if (filteredData !== null) {
                setStatusFilter(true);
            } else {
                setStatusFilter(false);
            }
        };

        getFilteredData();
    }, [unfilteredDataList, searchText]);

    return { statusFilter, filteredData };
};

export default useFilteredData;
