import settingsConfig from "../../fuse-configs/settingsConfig";

const useActualProjectName = () => {
    return settingsConfig.layout.project;
};

export default useActualProjectName;
