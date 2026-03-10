/**
 * Swarm Control Service
 * Provides SignalR communication with backend which bridges to ROS Task Orchestrator
 * Handles robot deployment, task assignment, and real-time status updates including sensor data
 */

import * as signalR from '@microsoft/signalr';

// Types
export interface RobotPosition {
  x: number;
  y: number;
  z: number;
}

export interface LidarSensorData {
  ranges: number[];
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  timestamp: string | null;
}

export interface RobotSensors {
  lidar: LidarSensorData;
}

export interface RobotStatus {
  id: string;
  role: string;
  position: RobotPosition;
  orientation: { theta: number };
  velocity: { linear: number; angular: number };
  status: 'active' | 'idle' | 'avoiding' | 'stuck' | 'error';
  threat_level: number;
  sensors?: RobotSensors;
}

export interface TaskStatus {
  task_id: string | null;
  task_type: string | null;
  status: 'idle' | 'initializing' | 'running' | 'paused' | 'completed' | 'failed' | 'stopped';
  progress: number;
}

export interface SwarmStatusUpdate {
  type: 'status_update';
  timestamp: string;
  task: TaskStatus;
  robots: RobotStatus[];
  robot_count: number;
  emergency_stop: boolean;
  collisions: number;
}

export interface Waypoint {
  x: number;
  y: number;
  theta?: number;
}

export type FormationType = 'line' | 'triangle' | 'circle' | 'square' | 'v' | 'diamond';
export type MovementMode = 'static' | 'moving' | 'adaptive';
export type LeaderMode = 'waypoint' | 'manual' | 'circular' | 'square' | 'figure8' | 'random';
export type SpawnPattern = 'grid' | 'circle' | 'line';
export type ObstacleDensity = 'low' | 'medium' | 'high';

export interface RobotDeploymentInfo {
  id: string;
  name: string;
}

// Goal configuration interfaces
export interface Position2D {
  x: number;
  y: number;
}

export interface FollowLeaderConfig {
  mode: LeaderMode;
  waypoints?: Waypoint[];
  radius?: number;       // For circular path
  sideLength?: number;   // For square path
}

export interface FormationConfig {
  type: FormationType;
  movementMode: MovementMode;
  center: Position2D;
  spacing: number;
  target?: Position2D;   // For moving mode
}

export interface TransportConfig {
  objectPosition: Position2D;
  objectSize: number;
  targetLocation: Position2D;
}

// Callback types
export type StatusCallback = (status: SwarmStatusUpdate) => void;
export type SensorCallback = (robotId: string, sensors: RobotSensors) => void;
export type ErrorCallback = (error: string) => void;
export type ConnectionCallback = (connected: boolean) => void;
export type RosConnectionCallback = (connected: boolean) => void;

/**
 * SwarmService - Main service for controlling robot swarm via SignalR
 * Connects to .NET backend which bridges to ROS WebSocket
 */
class SwarmService {
  private connection: signalR.HubConnection | null = null;
  private hubUrl: string;

  // Callbacks
  private onStatusUpdate: StatusCallback | null = null;
  private onSensorUpdate: SensorCallback | null = null;
  private onError: ErrorCallback | null = null;
  private onConnectionChange: ConnectionCallback | null = null;
  private onRosConnectionChange: RosConnectionCallback | null = null;

  // State
  private isConnected: boolean = false;
  private isRosConnected: boolean = false;
  private lastStatus: SwarmStatusUpdate | null = null;

  constructor(hubUrl: string = '/hubs/robot') {
    this.hubUrl = hubUrl;
  }

  // ==================== Connection Management ====================

  /**
   * Connect to SignalR Hub
   */
  async connect(): Promise<void> {
    try {
      console.log(`Connecting to SignalR hub at ${this.hubUrl}...`);

      this.connection = new signalR.HubConnectionBuilder()
        .withUrl(this.hubUrl)
        .withAutomaticReconnect([0, 2000, 5000, 10000, 30000])
        .configureLogging(signalR.LogLevel.Information)
        .build();

      // Set up event handlers
      this.setupEventHandlers();

      // Start connection
      await this.connection.start();

      this.isConnected = true;
      this.onConnectionChange?.(true);
      console.log('SignalR connected successfully');

    } catch (error) {
      console.error('Failed to connect to SignalR:', error);
      this.onError?.('Failed to connect to server');
      throw error;
    }
  }

  /**
   * Set up SignalR event handlers
   */
  private setupEventHandlers(): void {
    if (!this.connection) return;

    // Swarm status updates from ROS
    this.connection.on('SwarmStatusUpdate', (json: string) => {
      try {
        const status = JSON.parse(json) as SwarmStatusUpdate;
        this.lastStatus = status;
        this.onStatusUpdate?.(status);
      } catch (e) {
        console.error('Failed to parse swarm status:', e);
      }
    });

    // Individual robot sensor updates
    this.connection.on('RobotSensorUpdate', (robotId: string, json: string) => {
      try {
        const sensors = JSON.parse(json) as RobotSensors;
        this.onSensorUpdate?.(robotId, sensors);
      } catch (e) {
        console.error('Failed to parse sensor data:', e);
      }
    });

    // ROS connection status
    this.connection.on('RosConnectionChanged', (data: { connected: boolean; timestamp: string }) => {
      this.isRosConnected = data.connected;
      this.onRosConnectionChange?.(data.connected);
      console.log(`ROS connection status: ${data.connected ? 'connected' : 'disconnected'}`);
    });

    // Task events
    this.connection.on('TaskEvent', (json: string) => {
      console.log('Task event:', json);
    });

    // Emergency stop
    this.connection.on('EmergencyStop', (json: string) => {
      console.warn('Emergency stop activated:', json);
    });

    // Robot deployment response
    this.connection.on('RobotDeploymentResponse', (json: string) => {
      console.log('Deployment response:', json);
    });

    // Connection state changes
    this.connection.onreconnecting(() => {
      this.isConnected = false;
      this.onConnectionChange?.(false);
      console.log('SignalR reconnecting...');
    });

    this.connection.onreconnected(() => {
      this.isConnected = true;
      this.onConnectionChange?.(true);
      console.log('SignalR reconnected');
    });

    this.connection.onclose(() => {
      this.isConnected = false;
      this.onConnectionChange?.(false);
      console.log('SignalR connection closed');
    });
  }

  /**
   * Disconnect from SignalR
   */
  async disconnect(): Promise<void> {
    if (this.connection) {
      await this.connection.stop();
      this.connection = null;
    }
    this.isConnected = false;
  }

  // ==================== Event Handlers ====================

  /**
   * Set callback for status updates (called at 10Hz)
   */
  setStatusCallback(callback: StatusCallback): void {
    this.onStatusUpdate = callback;
  }

  /**
   * Set callback for individual robot sensor updates
   */
  setSensorCallback(callback: SensorCallback): void {
    this.onSensorUpdate = callback;
  }

  /**
   * Set callback for errors
   */
  setErrorCallback(callback: ErrorCallback): void {
    this.onError = callback;
  }

  /**
   * Set callback for SignalR connection state changes
   */
  setConnectionCallback(callback: ConnectionCallback): void {
    this.onConnectionChange = callback;
  }

  /**
   * Set callback for ROS connection state changes
   */
  setRosConnectionCallback(callback: RosConnectionCallback): void {
    this.onRosConnectionChange = callback;
  }

  // ==================== Robot Deployment ====================

  /**
   * Spawn robots in Gazebo
   * @param count Number of robots to spawn (1-20)
   * @param pattern Spawn pattern: 'grid', 'circle', 'line'
   */
  async spawnRobots(count: number, pattern: SpawnPattern = 'grid'): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('SpawnRobots', count, pattern);
  }

  /**
   * Spawn robots with specific IDs from database
   * @param robots Array of robot deployment info with IDs and names
   * @param pattern Spawn pattern: 'grid', 'circle', 'line'
   */
  async spawnRobotsWithIds(robots: RobotDeploymentInfo[], pattern: SpawnPattern = 'grid'): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('SpawnRobotsWithIds', robots, pattern);
  }

  /**
   * Delete robots from simulation
   * @param robotIds Specific robot IDs to delete (empty = delete all)
   */
  async deleteRobots(robotIds?: string[]): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('DeleteRobots', robotIds || []);
  }

  // ==================== Task Assignment ====================

  /**
   * Start Follow-the-Leader task
   */
  async startFollowLeader(
    robotCount: number,
    leaderMode: LeaderMode,
    config?: FollowLeaderConfig
  ): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>(
      'StartFollowLeader',
      robotCount,
      leaderMode,
      config || { mode: leaderMode, waypoints: [] }
    );
  }

  /**
   * Start Formation task
   */
  async startFormation(
    robotCount: number,
    formationType: FormationType,
    movementMode: MovementMode = 'static',
    config?: FormationConfig
  ): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>(
      'StartFormation',
      robotCount,
      formationType,
      movementMode,
      config || { type: formationType, movementMode, center: { x: 0, y: 0 }, spacing: 1.0 }
    );
  }

  /**
   * Start Collaborative Transport task
   */
  async startTransport(
    robotCount: number,
    targetX: number,
    targetY: number,
    config?: TransportConfig
  ): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>(
      'StartTransport',
      robotCount,
      targetX,
      targetY,
      config || { objectPosition: { x: 0, y: 0 }, objectSize: 0.5, targetLocation: { x: targetX, y: targetY } }
    );
  }

  /**
   * Stop current task
   */
  async stopTask(): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('StopTask');
  }

  /**
   * Emergency stop - immediately halts all robots
   */
  async emergencyStop(): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('EmergencyStop');
  }

  // ==================== Leader Control (Manual Mode) ====================

  /**
   * Control leader robot directly (for manual mode)
   * @param linearVelocity Forward/backward velocity (-0.26 to 0.26 m/s)
   * @param angularVelocity Rotation velocity (-1.82 to 1.82 rad/s)
   */
  async controlLeader(linearVelocity: number, angularVelocity: number): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>(
      'ControlLeader',
      linearVelocity,
      angularVelocity
    );
  }

  // ==================== Environment Control ====================

  /**
   * Configure environment (spawn obstacles)
   * @param density Obstacle density: 'low', 'medium', 'high'
   */
  async spawnObstacles(density: ObstacleDensity = 'medium'): Promise<boolean> {
    if (!this.connection) throw new Error('Not connected');
    return await this.connection.invoke<boolean>('SpawnObstacles', density);
  }

  // ==================== Status Queries ====================

  /**
   * Get ROS connection status
   */
  async getRosConnectionStatus(): Promise<boolean> {
    if (!this.connection) return false;
    return await this.connection.invoke<boolean>('GetRosConnectionStatus');
  }

  /**
   * Get last known status (cached)
   */
  getLastStatus(): SwarmStatusUpdate | null {
    return this.lastStatus;
  }

  /**
   * Check if connected to SignalR
   */
  isConnectedToServer(): boolean {
    return this.isConnected;
  }

  /**
   * Check if ROS is connected (via backend)
   */
  isRosConnectionActive(): boolean {
    return this.isRosConnected;
  }
}

// Determine hub URL based on environment
const getHubUrl = (): string => {
  // In production, use relative URL
  if (process.env.NODE_ENV === 'production') {
    return '/hubs/robot';
  }
  // In development, connect to backend
  return process.env.REACT_APP_API_URL
    ? `${process.env.REACT_APP_API_URL}/hubs/robot`
    : 'http://localhost:44336/hubs/robot';
};

// Export singleton instance
export const swarmService = new SwarmService(getHubUrl());
export default swarmService;
