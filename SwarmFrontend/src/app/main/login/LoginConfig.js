import { authRoles } from "app/auth";
import { Navigate } from "react-router-dom";
import Login from "./Login";

const LoginConfig = {
    settings: {
        layout: {
            config: {
                navbar: {
                    display: false,
                },
                toolbar: {
                    display: false,
                },
                footer: {
                    display: false,
                },
                leftSidePanel: {
                    display: false,
                },
                rightSidePanel: {
                    display: false,
                },
            },
        },
    },
    auth: authRoles.onlyGuest,
    routes: [
        {
            path: "/login",
            element: <Login />,
        },
        {
            path: "/register",
            element: <Navigate to="/login" replace />,
        },
    ],
};

export default LoginConfig;
