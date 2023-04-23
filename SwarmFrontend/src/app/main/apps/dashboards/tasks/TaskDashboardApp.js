import FusePageSimple from '@fuse/core/FusePageSimple';
import { styled } from '@mui/material/styles';
import withReducer from 'app/store/withReducer';
import _ from '@lodash';
import { useEffect, useRef, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import TaskDashboardAppHeader from './TaskDashboardAppHeader';
import reducer from './store';
import { getWidgets, selectWidgets } from './store/widgetsSlice';
import HomeTab from './tabs/HomeTab';
import OneSignal from 'react-onesignal';
import useLoggedUserId from "../../../../shared-components/hooks/useLoggedUserId";


const Root = styled(FusePageSimple)(({ theme }) => ({
  '& .FusePageSimple-header': {
    minHeight: 100,
    height: 100,
    [theme.breakpoints.up('lg')]: {
      marginRight: 12,
      borderBottomRightRadius: 20,
    },
  },
  '& .FusePageSimple-toolbar': {
    minHeight: 56,
    height: 56,
    alignItems: 'flex-end',
  },
  '& .FusePageSimple-rightSidebar': {
    width: 288,
    border: 0,
    padding: '12px 0',
  },
  '& .FusePageSimple-content': {
    maxHeight: '100%',
    '& canvas': {
      maxHeight: '100%',
    },
  },
}));






function TaskDashboardApp() {
  const dispatch = useDispatch();
  const widgets = useSelector(selectWidgets);

  const pageLayout = useRef(null);
  const [tabValue, setTabValue] = useState(0);

  const onHandleTag = (tag) =>{
    console.log(tag);
    OneSignal.sendTag('user', tag).then(()=>{
      console.log("Tagged");
      console.log(tag);
    });
  }

  useEffect(async () => {
    onHandleTag(useLoggedUserId().toString());
    dispatch(getWidgets());
  }, [dispatch]);

  function handleChangeTab(event, value) {
    setTabValue(value);
  }

  if (_.isEmpty(widgets)) {
    return null;
  }

  return (
    <Root
      header={<TaskDashboardAppHeader pageLayout={pageLayout} />}
      /*contentToolbar={
        <Tabs
          value={tabValue}
          onChange={handleChangeTab}
          indicatorColor="secondary"
          textColor="inherit"
          variant="scrollable"
          scrollButtons={false}
          className="w-full px-24 -mx-4 min-h-40"
          classes={{ indicator: 'flex justify-center bg-transparent w-full h-full' }}
          TabIndicatorProps={{
            children: (
              <Box
                sx={{ bgcolor: 'text.disabled' }}
                className="w-full h-full rounded-full opacity-20"
              />
            ),
          }}
        >
          <Tab
            className="text-14 font-semibold min-h-40 min-w-64 mx-4 px-12"
            disableRipple
            label="Home"
          />
          <Tab
            className="text-14 font-semibold min-h-40 min-w-64 mx-4 px-12"
            disableRipple
            label="Budget Summary"
          />
          <Tab
            className="text-14 font-semibold min-h-40 min-w-64 mx-4 px-12"
            disableRipple
            label="Team Members"
          />
        </Tabs>
      }*/
      content={
        <div className="p-12 lg:ltr:pr-0 lg:rtl:pl-0">
          <HomeTab />
          {/*{tabValue === 0 && <HomeTab />}*/}
          {/*{tabValue === 1 && <BudgetSummaryTab />}
          {tabValue === 2 && <TeamMembersTab />}*/}
        </div>
      }/*
      rightSidebarContent={<TaskDashboardAppSidebar />}*/
      ref={pageLayout}
    />
  );
}

export default withReducer('taskDashboardApp', reducer)(TaskDashboardApp);
