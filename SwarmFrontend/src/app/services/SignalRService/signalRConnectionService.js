import { HubConnectionBuilder, LogLevel } from '@microsoft/signalr';
import jwtService from 'app/services/jwtService';
import { URL } from 'app/constants/constants';

const singletonInstance = (function () {
    // Store multiple connections by group name
    const connections = {};
    // Store connection promises for each connection
    const connectionPromises = {};

    function createInstance(groupNames) {
        const accessToken = jwtService.getAccessToken();

        // Create a unique key for this connection
        const connectionKey = groupNames || 'default';

        // If we already have a connection for this group, return it
        if (connections[connectionKey]) {
            console.log(`Reusing existing SignalR connection for group: ${connectionKey}`);
            return connections[connectionKey];
        }

        console.log(`Creating new SignalR connection for group: ${connectionKey}`);

        const connection = new HubConnectionBuilder()
            .withUrl(`${URL}/hubs/robot${groupNames ? `?group=${groupNames}` : ''}`, {
                accessTokenFactory() {
                    return accessToken;
                }
            })
            .withAutomaticReconnect()
            .configureLogging(LogLevel.Warning)
            .build();

        // Store the connection
        connections[connectionKey] = connection;

        // Start the connection and store the promise
        connectionPromises[connectionKey] = connection.start()
            .then(() => {
                console.log(`SignalR connection established successfully for group: ${connectionKey}`);

                // Register connection event handlers
                connection.onreconnecting((error) => {
                    console.log(`SignalR reconnecting for group ${connectionKey}:`, error);
                });

                connection.onclose((error) => {
                    console.log(`SignalR connection closed for group ${connectionKey}:`, error);
                    // Remove the connection from our store when it's closed
                    delete connections[connectionKey];
                    delete connectionPromises[connectionKey];
                });

                return connection;
            })
            .catch(err => {
                console.error(`Error starting SignalR connection for group ${connectionKey}:`, err);
                // Remove the connection from our store if it fails to start
                delete connections[connectionKey];
                delete connectionPromises[connectionKey];
                throw err;
            });

        return connection;
    }

    const getConnectionPromise = (groupNames) => {
        const connectionKey = groupNames || 'default';
        return connectionPromises[connectionKey] || Promise.reject(new Error(`No connection found for ${connectionKey}`));
    };

    const stopConnection = (groupNames) => {
        const connectionKey = groupNames || 'default';
        if (connections[connectionKey]) {
            connections[connectionKey].stop();
            delete connections[connectionKey];
            console.log(`Stopped SignalR connection for group: ${connectionKey}`);
        }
    };

    const stopAllConnections = () => {
        Object.keys(connections).forEach(key => {
            connections[key].stop();
            console.log(`Stopped SignalR connection for group: ${key}`);
        });
        // Clear all connections
        Object.keys(connections).forEach(key => {
            delete connections[key];
        });
    };

    return {
        createConnectionBuilder: createInstance,
        getConnectionPromise,
        stopConnection,
        stopAllConnections
    };
})();

export default singletonInstance;
