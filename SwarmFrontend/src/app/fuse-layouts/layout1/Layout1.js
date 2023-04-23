import FuseDialog from '@fuse/core/FuseDialog';
import { styled } from '@mui/material/styles';
import FuseMessage from '@fuse/core/FuseMessage';
import FuseSuspense from '@fuse/core/FuseSuspense';
import AppContext from 'app/AppContext';
import { memo, useContext } from 'react';
import { useSelector } from 'react-redux';
import {useLocation, useRoutes} from 'react-router-dom';
import LeftSideLayout1 from './components/LeftSideLayout1';
import NavbarWrapperLayout1 from './components/NavbarWrapperLayout1';
import RightSideLayout1 from './components/RightSideLayout1';
import ToolbarLayout1 from './components/ToolbarLayout1';
import GoToTaskOptionsButton from "../../main/apps/dashboards/tasks/tabs/GoToTaskOptionsButton";
import settingsConfig from "../../fuse-configs/settingsConfig";

const Root = styled('div')(({ config }) => ({

  ...(config.mode === 'boxed' && {
    clipPath: 'inset(0)',
    maxWidth: `${config.containerWidth}px`,
    margin: '0 auto',
    boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
  }),
  ...(config.mode === 'container' && {
    '& .container': {
      maxWidth: `${config.containerWidth}px`,
      width: '100%',
      margin: '0 auto',
    },
  }),
}));


function Layout1(props) {

  const actualPageLocation = useLocation().pathname;
  const config = useSelector(({ fuse }) => fuse.settings.current.layout.config);
  const appContext = useContext(AppContext);
  const { routes } = appContext;


  return (
      <>

        <Root id="fuse-layout" config={config} className="w-full flex">
          {config.leftSidePanel.display && <LeftSideLayout1 />}

          <div className="flex flex-auto min-w-0">
            {config.navbar.display && config.navbar.position === 'left' && <NavbarWrapperLayout1 />}

            <main id="fuse-main" className="flex flex-col flex-auto min-h-screen min-w-0 relative z-10">
              {config.toolbar.display && (
                  <ToolbarLayout1 className={config.toolbar.style === 'fixed' && 'sticky top-0'} />
              )}


              <div className="flex flex-col flex-auto min-h-0 relative z-10">
                <FuseDialog />

                <FuseSuspense>{useRoutes(routes)}</FuseSuspense>

                {props.children}
              </div>

            </main>

            {config.navbar.display && config.navbar.position === 'right' && <NavbarWrapperLayout1 />}

          </div>

          {config.rightSidePanel.display && <RightSideLayout1 />}

          <FuseMessage />

        </Root>

        {settingsConfig.layout.project === "task" && actualPageLocation !== "/login" ?(

            <GoToTaskOptionsButton/>

        ): null}

      </>
  );
}

export default memo(Layout1);
