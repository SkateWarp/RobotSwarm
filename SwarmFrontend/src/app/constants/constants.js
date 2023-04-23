import settingsConfig from "../fuse-configs/settingsConfig";

export let URL = window.location.href;
export let LOGO = "assets/images/logos/alterna.png";

switch (settingsConfig.layout.project) {
    case "task":
        LOGO = "assets/images/logos/alterna.png";
        URL = "https://atapi.alternard.com";
        break;
    case "GTS":
        LOGO = "assets/images/logos/alterna.png";
        // FOR DEV
        URL = "https://swarm.msk.do";
        // FOR LOCALHOST
        // URL = 'https://localhost:44337';
        // URL = 'http://10.0.0.96:5017';
        break;
    case "GTS-swedish":
        LOGO = "assets/images/logos/swedish.png";
        // FOR DEV
        // URL = "https://devswapi.alternard.com";
        // FOR LOCALHOST
        URL = "https://localhost:44337";
        // URL = 'http://10.0.0.96:5017';
        break;

    case "panelTemp":
        LOGO = "assets/images/logos/alterna.png";

        URL = "https://devfapi.alternard.com";

        // FOR DEV
        // URL = "https://panelapi.alternard.com";
        // URL = 'https://localhost:44337';

        break;
}
