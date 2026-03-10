/**
 * React Hook for Swarm Control
 * Provides easy access to robot deployment and task management via SignalR
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import swarmService, {
  SwarmStatusUpdate,
  RobotStatus,
  RobotSensors,
  TaskStatus,
  Waypoint,
  FormationType,
  MovementMode,
  LeaderMode,
  SpawnPattern,
  ObstacleDensity
} from '../services/SwarmService';

export interface UseSwarmControlReturn {
  // Connection state
  isConnected: boolean;
  isRosConnected: boolean;
  connect: () => Promise<void>;
  disconnect: () => Promise<void>;

  // Status
  robots: RobotStatus[];
  taskStatus: TaskStatus | null;
  robotCount: number;
  isEmergencyStopped: boolean;
  lastUpdate: string | null;

  // Sensor data (keyed by robot ID)
  sensorData: Map<string, RobotSensors>;

  // Deployment
  spawnRobots: (count: number, pattern?: SpawnPattern) => Promise<void>;
  deleteRobots: (robotIds?: string[]) => Promise<void>;
  deleteAllRobots: () => Promise<void>;

  // Tasks
  startFollowLeader: (options?: {
    waypoints?: Waypoint[];
    leaderMode?: LeaderMode;
  }) => Promise<void>;
  startFormation: (options?: {
    formationType?: FormationType;
    movementMode?: MovementMode;
  }) => Promise<void>;
  startTransport: (targetLocation: { x: number; y: number }) => Promise<void>;
  stopTask: () => Promise<void>;
  emergencyStop: () => Promise<void>;

  // Leader control
  controlLeader: (linear: number, angular: number) => Promise<void>;

  // Environment
  spawnObstacles: (density: ObstacleDensity) => Promise<void>;

  // Error handling
  error: string | null;
  clearError: () => void;
}

export function useSwarmControl(): UseSwarmControlReturn {
  // State
  const [isConnected, setIsConnected] = useState(false);
  const [isRosConnected, setIsRosConnected] = useState(false);
  const [robots, setRobots] = useState<RobotStatus[]>([]);
  const [taskStatus, setTaskStatus] = useState<TaskStatus | null>(null);
  const [robotCount, setRobotCount] = useState(0);
  const [isEmergencyStopped, setIsEmergencyStopped] = useState(false);
  const [lastUpdate, setLastUpdate] = useState<string | null>(null);
  const [sensorData, setSensorData] = useState<Map<string, RobotSensors>>(new Map());
  const [error, setError] = useState<string | null>(null);

  // Refs to track mounted state
  const isMounted = useRef(true);

  // Setup callbacks on mount
  useEffect(() => {
    isMounted.current = true;

    // Status update callback
    swarmService.setStatusCallback((status: SwarmStatusUpdate) => {
      if (!isMounted.current) return;

      setRobots(status.robots);
      setTaskStatus(status.task);
      setRobotCount(status.robot_count);
      setIsEmergencyStopped(status.emergency_stop);
      setLastUpdate(status.timestamp);
    });

    // Sensor update callback
    swarmService.setSensorCallback((robotId: string, sensors: RobotSensors) => {
      if (!isMounted.current) return;

      setSensorData(prev => {
        const newMap = new Map(prev);
        newMap.set(robotId, sensors);
        return newMap;
      });
    });

    // Error callback
    swarmService.setErrorCallback((errorMsg: string) => {
      if (!isMounted.current) return;
      setError(errorMsg);
    });

    // Connection callback (SignalR)
    swarmService.setConnectionCallback((connected: boolean) => {
      if (!isMounted.current) return;
      setIsConnected(connected);
    });

    // ROS connection callback
    swarmService.setRosConnectionCallback((connected: boolean) => {
      if (!isMounted.current) return;
      setIsRosConnected(connected);
    });

    // Check if already connected
    setIsConnected(swarmService.isConnectedToServer());
    setIsRosConnected(swarmService.isRosConnectionActive());

    return () => {
      isMounted.current = false;
    };
  }, []);

  // Connection
  const connect = useCallback(async () => {
    try {
      await swarmService.connect();
    } catch (err) {
      setError('Failed to connect to server');
    }
  }, []);

  const disconnect = useCallback(async () => {
    await swarmService.disconnect();
  }, []);

  // Deployment
  const spawnRobots = useCallback(async (count: number, pattern: SpawnPattern = 'grid') => {
    try {
      await swarmService.spawnRobots(count, pattern);
    } catch (err) {
      setError('Failed to spawn robots');
    }
  }, []);

  const deleteRobots = useCallback(async (robotIds?: string[]) => {
    try {
      await swarmService.deleteRobots(robotIds);
    } catch (err) {
      setError('Failed to delete robots');
    }
  }, []);

  const deleteAllRobots = useCallback(async () => {
    await deleteRobots();
  }, [deleteRobots]);

  // Tasks
  const startFollowLeader = useCallback(async (options?: {
    waypoints?: Waypoint[];
    leaderMode?: LeaderMode;
  }) => {
    try {
      await swarmService.startFollowLeader(
        robotCount,
        options?.leaderMode || 'waypoint',
        { mode: options?.leaderMode || 'waypoint', waypoints: options?.waypoints || [] }
      );
    } catch (err) {
      setError('Failed to start follow-leader task');
    }
  }, [robotCount]);

  const startFormation = useCallback(async (options?: {
    formationType?: FormationType;
    movementMode?: MovementMode;
  }) => {
    try {
      await swarmService.startFormation(
        robotCount,
        options?.formationType || 'triangle',
        options?.movementMode || 'static'
      );
    } catch (err) {
      setError('Failed to start formation task');
    }
  }, [robotCount]);

  const startTransport = useCallback(async (targetLocation: { x: number; y: number }) => {
    try {
      await swarmService.startTransport(
        robotCount,  // Use all available robots - task adapts dynamically to N robots
        targetLocation.x,
        targetLocation.y
      );
    } catch (err) {
      setError('Failed to start transport task');
    }
  }, [robotCount]);

  const stopTask = useCallback(async () => {
    try {
      await swarmService.stopTask();
    } catch (err) {
      setError('Failed to stop task');
    }
  }, []);

  const emergencyStop = useCallback(async () => {
    try {
      await swarmService.emergencyStop();
    } catch (err) {
      setError('Failed to execute emergency stop');
    }
  }, []);

  // Leader control
  const controlLeader = useCallback(async (linear: number, angular: number) => {
    try {
      await swarmService.controlLeader(linear, angular);
    } catch (err) {
      // Don't set error for control commands (too frequent)
      console.error('Control command failed:', err);
    }
  }, []);

  // Environment
  const spawnObstacles = useCallback(async (density: ObstacleDensity) => {
    try {
      await swarmService.spawnObstacles(density);
    } catch (err) {
      setError('Failed to spawn obstacles');
    }
  }, []);

  // Error handling
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    // Connection
    isConnected,
    isRosConnected,
    connect,
    disconnect,

    // Status
    robots,
    taskStatus,
    robotCount,
    isEmergencyStopped,
    lastUpdate,

    // Sensor data
    sensorData,

    // Deployment
    spawnRobots,
    deleteRobots,
    deleteAllRobots,

    // Tasks
    startFollowLeader,
    startFormation,
    startTransport,
    stopTask,
    emergencyStop,

    // Leader control
    controlLeader,

    // Environment
    spawnObstacles,

    // Error
    error,
    clearError
  };
}

export default useSwarmControl;
