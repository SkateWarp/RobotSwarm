import FuseSplashScreen from "@fuse/core/FuseSplashScreen";
import jwtService from "app/services/jwtService";
import { Component } from "react";
import { connect } from "react-redux";
import { bindActionCreators } from "@reduxjs/toolkit";
import { hideMessage, showMessage } from "app/store/fuse/messageSlice";

import { setUserData, logoutUser } from "./store/userSlice";

class Auth extends Component {
    state = {
        waitAuthCheck: true,
    };

    componentDidMount() {
        return Promise.all([this.jwtCheck()]).then(() => {
            this.setState({ waitAuthCheck: false });
        });
    }

    jwtCheck = () =>
        new Promise((resolve) => {
            jwtService.on("onAutoLogin", () => {
                // this.props.showMessage({ message: 'Login autorizado.' });

                /**
                 * Sign in and retrieve user data from Api
                 */
                jwtService
                    .signInWithToken()
                    .then((user) => {
                        this.props.setUserData(user);

                        resolve();

                        // this.props.showMessage({ message: 'Login autorizado.' });
                    })
                    .catch((error) => {
                        this.props.showMessage({ message: error.message });

                        resolve();
                    });
            });

            jwtService.on("onAutoLogout", (message) => {
                if (message) {
                    this.props.showMessage({ message });
                }

                this.props.logout();

                resolve();
            });

            jwtService.on("onNoAccessToken", () => {
                resolve();
            });

            jwtService.init();

            return Promise.resolve();
        });

    render() {
        return this.state.waitAuthCheck ? <FuseSplashScreen /> : <>{this.props.children}</>;
    }
}

function mapDispatchToProps(dispatch) {
    return bindActionCreators(
        {
            logout: logoutUser,
            setUserData,
            showMessage,
            hideMessage,
        },
        dispatch
    );
}

export default connect(null, mapDispatchToProps)(Auth);
