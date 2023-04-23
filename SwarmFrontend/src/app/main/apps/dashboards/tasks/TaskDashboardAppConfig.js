import { lazy } from 'react';
import authRoles from '../../../../auth/authRoles';

const TaskDashboardApp = lazy(() => import('./TaskDashboardApp'));

const TaskDashboardAppConfig = {
  settings: {
    layout: {
      config: {},
    },
  },
  auth: authRoles.user,
  routes: [
    {
      path: 'apps/dashboard/tasks',
      element: <TaskDashboardApp />,
    },
  ],
};

export default TaskDashboardAppConfig;
