import { HubConnectionBuilder, LogLevel } from '@microsoft/signalr';
import jwtService from 'app/services/jwtService';
import { URL } from 'app/constants/constants';

const singletonInstance = (function () {

    let instance;

    function createInstance(groupNames) {

        const accessToken = jwtService.getAccessToken();

        return new HubConnectionBuilder().withUrl(`${URL}/hubs/robot${groupNames ? `?group=${groupNames}` : ''}`, {

            accessTokenFactory() {

                return accessToken;
            }
        }).withAutomaticReconnect().configureLogging(LogLevel.None).build();
    }


    const stopConnection = () => {

        instance.stop();
    };


    return {

        createConnectionBuilder(groupNames) {

            function start() {
                instance = createInstance(groupNames);

                try {
                    instance.start();
                } catch (err) {
                }
            }

            if (!instance) {
                start();
            } else {
                instance.stop();
                start();
            }

            return instance;
        }, stopConnection, createConnectionBuilder2(groupNames) {
            const instance2 = createInstance(groupNames);
            instance2.start();
            return instance2;
        }
    };
})();

export default singletonInstance;
