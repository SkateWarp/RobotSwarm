/**
 * React Hook for Swarm Control
 * Provides easy access to robot deployment and task management via SignalR
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import swarmService from '../services/SwarmService';

export function useSwarmControl() {
  // State
  const [isConnected, setIsConnected] = useState(false);
  const [isRosConnected, setIsRosConnected] = useState(false);
  const [robots, setRobots] = useState([]);
  const [taskStatus, setTaskStatus] = useState(null);
  const [robotCount, setRobotCount] = useState(0);
  const [isEmergencyStopped, setIsEmergencyStopped] = useState(false);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [sensorData, setSensorData] = useState(new Map());
  const [error, setError] = useState(null);

  // Refs to track mounted state
  const isMounted = useRef(true);

  // Setup callbacks on mount
  useEffect(() => {
    isMounted.current = true;

    // Status update callback
    swarmService.setStatusCallback((status) => {
      if (!isMounted.current) return;

      setRobots(status.robots);
      setTaskStatus(status.task);
      setRobotCount(status.robot_count);
      setIsEmergencyStopped(status.emergency_stop);
      setLastUpdate(status.timestamp);
    });

    // Sensor update callback
    swarmService.setSensorCallback((robotId, sensors) => {
      if (!isMounted.current) return;

      setSensorData(prev => {
        const newMap = new Map(prev);
        newMap.set(robotId, sensors);
        return newMap;
      });
    });

    // Error callback
    swarmService.setErrorCallback((errorMsg) => {
      if (!isMounted.current) return;
      setError(errorMsg);
    });

    // Connection callback (SignalR)
    swarmService.setConnectionCallback((connected) => {
      if (!isMounted.current) return;
      setIsConnected(connected);
    });

    // ROS connection callback
    swarmService.setRosConnectionCallback((connected) => {
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
  const spawnRobots = useCallback(async (count, pattern = 'grid') => {
    try {
      await swarmService.spawnRobots(count, pattern);
    } catch (err) {
      setError('Failed to spawn robots');
    }
  }, []);

  const deleteRobots = useCallback(async (robotIds) => {
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
  const startFollowLeader = useCallback(async (options) => {
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

  const startFormation = useCallback(async (options) => {
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

  const startTransport = useCallback(async (targetLocation) => {
    try {
      await swarmService.startTransport(
        robotCount,
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
  const controlLeader = useCallback(async (linear, angular) => {
    try {
      await swarmService.controlLeader(linear, angular);
    } catch (err) {
      // Don't set error for control commands (too frequent)
      console.error('Control command failed:', err);
    }
  }, []);

  // Environment
  const spawnObstacles = useCallback(async (density) => {
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
