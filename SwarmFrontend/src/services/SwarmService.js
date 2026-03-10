/**
 * Swarm Control Service
 * Provides SignalR communication with backend which bridges to ROS Task Orchestrator
 * Handles robot deployment, task assignment, and real-time status updates including sensor data
 */

import * as signalR from '@microsoft/signalr';

/**
 * SwarmService - Main service for controlling robot swarm via SignalR
 * Connects to .NET backend which bridges to ROS WebSocket
 */
class SwarmService {
  constructor(hubUrl = '/hubs/robot') {
    this.connection = null;
    this.hubUrl = hubUrl;

    // Callbacks
    this.onStatusUpdate = null;
    this.onSensorUpdate = null;
    this.onError = null;
    this.onConnectionChange = null;
    this.onRosConnectionChange = null;

    // State
    this._isConnected = false;
    this._isRosConnected = false;
    this.lastStatus = null;
  }

  // ==================== Connection Management ====================

  async connect() {
    try {
      console.log(`Connecting to SignalR hub at ${this.hubUrl}...`);

      this.connection = new signalR.HubConnectionBuilder()
        .withUrl(this.hubUrl)
        .withAutomaticReconnect([0, 2000, 5000, 10000, 30000])
        .configureLogging(signalR.LogLevel.Information)
        .build();

      this.setupEventHandlers();
      await this.connection.start();

      this._isConnected = true;
      if (this.onConnectionChange) this.onConnectionChange(true);
      console.log('SignalR connected successfully');
    } catch (error) {
      console.error('Failed to connect to SignalR:', error);
      if (this.onError) this.onError('Failed to connect to server');
      throw error;
    }
  }

  setupEventHandlers() {
    if (!this.connection) return;

    // Swarm status updates from ROS
    this.connection.on('SwarmStatusUpdate', (json) => {
      try {
        const status = JSON.parse(json);
        this.lastStatus = status;
        if (this.onStatusUpdate) this.onStatusUpdate(status);
      } catch (e) {
        console.error('Failed to parse swarm status:', e);
      }
    });

    // Individual robot sensor updates
    this.connection.on('RobotSensorUpdate', (robotId, json) => {
      try {
        const sensors = JSON.parse(json);
        if (this.onSensorUpdate) this.onSensorUpdate(robotId, sensors);
      } catch (e) {
        console.error('Failed to parse sensor data:', e);
      }
    });

    // ROS connection status
    this.connection.on('RosConnectionChanged', (data) => {
      this._isRosConnected = data.connected;
      if (this.onRosConnectionChange) this.onRosConnectionChange(data.connected);
      console.log(`ROS connection status: ${data.connected ? 'connected' : 'disconnected'}`);
    });

    // Task events
    this.connection.on('TaskEvent', (json) => {
      console.log('Task event:', json);
    });

    // Emergency stop
    this.connection.on('EmergencyStop', (json) => {
      console.warn('Emergency stop activated:', json);
    });

    // Robot deployment response
    this.connection.on('RobotDeploymentResponse', (json) => {
      console.log('Deployment response:', json);
    });

    // Connection state changes
    this.connection.onreconnecting(() => {
      this._isConnected = false;
      if (this.onConnectionChange) this.onConnectionChange(false);
      console.log('SignalR reconnecting...');
    });

    this.connection.onreconnected(() => {
      this._isConnected = true;
      if (this.onConnectionChange) this.onConnectionChange(true);
      console.log('SignalR reconnected');
    });

    this.connection.onclose(() => {
      this._isConnected = false;
      if (this.onConnectionChange) this.onConnectionChange(false);
      console.log('SignalR connection closed');
    });
  }

  async disconnect() {
    if (this.connection) {
      await this.connection.stop();
      this.connection = null;
    }
    this._isConnected = false;
  }

  // ==================== Event Handlers ====================

  setStatusCallback(callback) {
    this.onStatusUpdate = callback;
  }

  setSensorCallback(callback) {
    this.onSensorUpdate = callback;
  }

  setErrorCallback(callback) {
    this.onError = callback;
  }

  setConnectionCallback(callback) {
    this.onConnectionChange = callback;
  }

  setRosConnectionCallback(callback) {
    this.onRosConnectionChange = callback;
  }

  // ==================== Robot Deployment ====================

  async spawnRobots(count, pattern = 'grid') {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('SpawnRobots', count, pattern);
  }

  async spawnRobotsWithIds(robots, pattern = 'grid') {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('SpawnRobotsWithIds', robots, pattern);
  }

  async deleteRobots(robotIds) {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('DeleteRobots', robotIds || []);
  }

  // ==================== Task Assignment ====================

  async startFollowLeader(robotCount, leaderMode, config) {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke(
      'StartFollowLeader',
      robotCount,
      leaderMode,
      config || { mode: leaderMode, waypoints: [] }
    );
  }

  async startFormation(robotCount, formationType, movementMode = 'static', config) {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke(
      'StartFormation',
      robotCount,
      formationType,
      movementMode,
      config || { type: formationType, movementMode, center: { x: 0, y: 0 }, spacing: 1.0 }
    );
  }

  async startTransport(robotCount, targetX, targetY, config) {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke(
      'StartTransport',
      robotCount,
      targetX,
      targetY,
      config || { objectPosition: { x: 0, y: 0 }, objectSize: 0.5, targetLocation: { x: targetX, y: targetY } }
    );
  }

  async stopTask() {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('StopTask');
  }

  async emergencyStop() {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('EmergencyStop');
  }

  // ==================== Leader Control (Manual Mode) ====================

  async controlLeader(linearVelocity, angularVelocity) {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('ControlLeader', linearVelocity, angularVelocity);
  }

  // ==================== Environment Control ====================

  async spawnObstacles(density = 'medium') {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke('SpawnObstacles', density);
  }

  // ==================== Status Queries ====================

  async getRosConnectionStatus() {
    if (!this.connection) return false;
    return await this.connection.invoke('GetRosConnectionStatus');
  }

  getLastStatus() {
    return this.lastStatus;
  }

  isConnectedToServer() {
    return this._isConnected;
  }

  isRosConnectionActive() {
    return this._isRosConnected;
  }
}

// Determine hub URL based on environment
const getHubUrl = () => {
  if (process.env.NODE_ENV === 'production') {
    return '/hubs/robot';
  }
  return process.env.REACT_APP_API_URL
    ? `${process.env.REACT_APP_API_URL}/hubs/robot`
    : 'http://localhost:44336/hubs/robot';
};

// Export singleton instance
export const swarmService = new SwarmService(getHubUrl());
export default swarmService;
