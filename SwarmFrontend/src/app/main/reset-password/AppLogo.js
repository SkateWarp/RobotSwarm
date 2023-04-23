import settingsConfig from "../../fuse-configs/settingsConfig";
import {LOGO} from "../../constants/constants";

const AppLogo = () => {

    if(settingsConfig.layout.project === "baldom" || settingsConfig.layout.project === "fakeBaldom")
        return <img className="w-128 m-32" src={LOGO} alt="logo"/>;

    if(settingsConfig.layout.project === "fraga" || settingsConfig.layout.project === "fakeFraga")
        return <img className="w-128 m-32" src={LOGO} alt="logo"/>;

    if (settingsConfig.layout.project === "task")
        return <img className="w-128 m-32" src={LOGO} alt="logo"/>;

    if (settingsConfig.layout.project === "GTS")
        return <img className="w-128 m-32" src={LOGO} alt="logo"/>;
};

export default AppLogo;
