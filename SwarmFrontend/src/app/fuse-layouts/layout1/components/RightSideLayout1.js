import QuickPanel from 'app/fuse-layouts/shared-components/quickPanel/QuickPanel';
import {memo} from 'react';
import NotificationPanel from "../../shared-components/notificationPanel/NotificationPanel";
import settingsConfig from "../../../fuse-configs/settingsConfig";
import QuickPanelFraga from "../../shared-components/quickPanelFraga/QuickPanel";

function RightSideLayout1() {
    return (
        <>
            {settingsConfig.layout.project==='baldom' && (<QuickPanel/>)}
            {settingsConfig.layout.project==='baldom' ? (<NotificationPanel/>):null}
            {settingsConfig.layout.project==='fraga' && (<QuickPanelFraga/>)}
        </>
    );
}

export default memo(RightSideLayout1);
