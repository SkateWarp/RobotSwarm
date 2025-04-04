import { HubConnectionBuilder, LogLevel, JsonHubProtocol } from '@microsoft/signalr';
import jwtService from 'app/services/jwtService';
import { URL } from 'app/constants/constants';

const singletonInstance = (function () {
    // Store multiple connections by group name
    const connections = {};

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

        // Create a custom JSON protocol that uses camelCase
        const jsonProtocol = new JsonHubProtocol();
        // Override the parse method to convert property names to camelCase
        const originalParse = jsonProtocol.parse;
        jsonProtocol.parse = function (message) {
            const parsed = originalParse.call(this, message);
            if (parsed && parsed.arguments && parsed.arguments.length > 0) {
                // Convert the first argument to camelCase
                parsed.arguments[0] = convertToCamelCase(parsed.arguments[0]);
            }
            return parsed;
        };

        const connection = new HubConnectionBuilder()
            .withUrl(`${URL}/hubs/robot${groupNames ? `?group=${groupNames}` : ''}`, {
                accessTokenFactory() {
                    return accessToken;
                }
            })
            .withHubProtocol(jsonProtocol)
            .withAutomaticReconnect()
            .configureLogging(LogLevel.Warning)
            .build();

        // Store the connection
        connections[connectionKey] = connection;

        // Start the connection
        connection.start()
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
                });
            })
            .catch(err => {
                console.error(`Error starting SignalR connection for group ${connectionKey}:`, err);
                // Remove the connection from our store if it fails to start
                delete connections[connectionKey];
            });

        return connection;
    }

    // Helper function to convert object keys to camelCase
    function convertToCamelCase(obj) {
        if (obj === null || typeof obj !== 'object') {
            return obj;
        }

        if (Array.isArray(obj)) {
            return obj.map(item => convertToCamelCase(item));
        }

        const result = {};
        for (const key in obj) {
            if (Object.prototype.hasOwnProperty.call(obj, key)) {
                const camelKey = key.charAt(0).toLowerCase() + key.slice(1);
                result[camelKey] = convertToCamelCase(obj[key]);
            }
        }
        return result;
    }

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
        stopConnection,
        stopAllConnections
    };
})();

export default singletonInstance;
