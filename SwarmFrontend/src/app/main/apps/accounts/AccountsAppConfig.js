import { lazy } from 'react';
import { Navigate } from 'react-router-dom';
import authRoles from '../../../auth/authRoles';

const AccountsApp = lazy(() => import('./AccountsApp'));

const AccountsAppConfig = {
  settings: {
    layout: {
      config: {},
    },
  },

  auth: authRoles.admin,

  routes: [
    {
      path: '/apps/accounts/:param',
      element: <AccountsApp />,
    },
    {
      path: '/apps/accounts',
      element: <Navigate to="/apps/accounts/all" />,
    },
  ],
};

export default AccountsAppConfig;
