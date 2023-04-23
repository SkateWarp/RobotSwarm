import { lazy } from 'react';

const ResetPasswordPage = lazy(() => import('./ResetPasswordPage'));

const ResetPasswordPageConfig = {
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
            path: '/reset-password',
            element: <ResetPasswordPage />,
        },
    ],
};

export default ResetPasswordPageConfig;
