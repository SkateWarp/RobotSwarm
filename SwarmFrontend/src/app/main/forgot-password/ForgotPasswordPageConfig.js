import { lazy } from 'react';

const ForgotPasswordPage = lazy(() => import('./ForgotPasswordPage'));

const ForgotPasswordPageConfig = {
  settings: {
    layout: {
      config: {
        navbar: {
          display: false
        },
        toolbar: {
          display: false
        },
        footer: {
          display: false
        },
        leftSidePanel: {
          display: false
        },
        rightSidePanel: {
          display: false
        }
      },
    },
  },
  routes: [
    {
      path: '/forgot-password',
      element: <ForgotPasswordPage />,
    },
  ],
};

export default ForgotPasswordPageConfig;
