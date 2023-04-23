import { lazy } from 'react';

const MailConfirmPage = lazy(() => import('./MailConfirmPage'));

const MailConfirmPageConfig = {
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
            path: 'pages/auth/mail-confirm',
            element: <MailConfirmPage />,
        },
    ],
};

export default MailConfirmPageConfig;
