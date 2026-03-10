/**
 * Swarm Control Panel Component
 * Complete UI for controlling robot deployment, task assignment, and monitoring
 */

import React, { useState, useEffect } from 'react';
import useSwarmControl from '../hooks/useSwarmControl';

// Styles (inline for simplicity - move to CSS file in production)
const styles = {
  container: {
    fontFamily: 'Arial, sans-serif',
    padding: '20px',
    maxWidth: '1200px',
    margin: '0 auto',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '20px',
  },
  connectionStatus: {
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
  },
  statusDot: {
    width: '12px',
    height: '12px',
    borderRadius: '50%',
  },
  section: {
    backgroundColor: '#f5f5f5',
    borderRadius: '8px',
    padding: '20px',
    marginBottom: '20px',
  },
  sectionTitle: {
    fontSize: '18px',
    fontWeight: 'bold',
    marginBottom: '15px',
    borderBottom: '2px solid #333',
    paddingBottom: '5px',
  },
  row: {
    display: 'flex',
    gap: '15px',
    marginBottom: '15px',
    flexWrap: 'wrap',
  },
  button: {
    padding: '10px 20px',
    borderRadius: '5px',
    border: 'none',
    cursor: 'pointer',
    fontWeight: 'bold',
    transition: 'background-color 0.2s',
  },
  primaryButton: {
    backgroundColor: '#007bff',
    color: 'white',
  },
  secondaryButton: {
    backgroundColor: '#6c757d',
    color: 'white',
  },
  dangerButton: {
    backgroundColor: '#dc3545',
    color: 'white',
  },
  successButton: {
    backgroundColor: '#28a745',
    color: 'white',
  },
  input: {
    padding: '8px 12px',
    borderRadius: '4px',
    border: '1px solid #ccc',
    fontSize: '14px',
  },
  select: {
    padding: '8px 12px',
    borderRadius: '4px',
    border: '1px solid #ccc',
    fontSize: '14px',
    minWidth: '150px',
  },
  label: {
    display: 'flex',
    flexDirection: 'column',
    gap: '5px',
    fontSize: '14px',
  },
  robotGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fill, minmax(200px, 1fr))',
    gap: '15px',
  },
  robotCard: {
    backgroundColor: 'white',
    borderRadius: '8px',
    padding: '15px',
    boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
  },
  progressBar: {
    width: '100%',
    height: '20px',
    backgroundColor: '#e0e0e0',
    borderRadius: '10px',
    overflow: 'hidden',
  },
  progressFill: {
    height: '100%',
    backgroundColor: '#28a745',
    transition: 'width 0.3s ease',
  },
  error: {
    backgroundColor: '#f8d7da',
    color: '#721c24',
    padding: '15px',
    borderRadius: '5px',
    marginBottom: '20px',
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  emergencyButton: {
    backgroundColor: '#dc3545',
    color: 'white',
    padding: '15px 30px',
    fontSize: '18px',
    fontWeight: 'bold',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    boxShadow: '0 4px 6px rgba(0,0,0,0.2)',
  },
};

const SwarmControlPanel = () => {
  const {
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

    // Sensor data
    sensorData,

    // Deployment
    spawnRobots,
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
  } = useSwarmControl();

  // Local state for form inputs
  const [robotCountInput, setRobotCountInput] = useState(5);
  const [spawnPattern, setSpawnPattern] = useState('grid');
  const [formationType, setFormationType] = useState('triangle');
  const [leaderMode, setLeaderMode] = useState('waypoint');
  const [obstacleDensity, setObstacleDensity] = useState('medium');
  const [targetX, setTargetX] = useState(7);
  const [targetY, setTargetY] = useState(7);

  // Waypoints for follow-leader
  const [waypoints, setWaypointsState] = useState([
    { x: 2, y: 2 },
    { x: 4, y: 2 },
    { x: 4, y: 4 },
    { x: 2, y: 4 },
  ]);

  // Auto-connect on mount
  useEffect(() => {
    connect();
    return () => disconnect();
  }, [connect, disconnect]);

  // Handler for adding waypoint
  const addWaypoint = () => {
    setWaypointsState([...waypoints, { x: 0, y: 0 }]);
  };

  // Handler for removing waypoint
  const removeWaypoint = (index) => {
    setWaypointsState(waypoints.filter((_, i) => i !== index));
  };

  // Handler for updating waypoint
  const updateWaypoint = (index, field, value) => {
    const newWaypoints = [...waypoints];
    newWaypoints[index][field] = value;
    setWaypointsState(newWaypoints);
  };

  return (
    <div style={styles.container}>
      {/* Header */}
      <div style={styles.header}>
        <h1>Swarm Control Panel</h1>
        <div style={styles.connectionStatus}>
          <div style={{ display: 'flex', gap: '20px', alignItems: 'center' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
              <div
                style={{
                  ...styles.statusDot,
                  backgroundColor: isConnected ? '#28a745' : '#dc3545',
                }}
              />
              <span>Server: {isConnected ? 'Connected' : 'Disconnected'}</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
              <div
                style={{
                  ...styles.statusDot,
                  backgroundColor: isRosConnected ? '#28a745' : '#ffc107',
                }}
              />
              <span>ROS: {isRosConnected ? 'Connected' : 'Waiting'}</span>
            </div>
          </div>
          <button
            style={{ ...styles.button, ...styles.secondaryButton }}
            onClick={isConnected ? disconnect : connect}
          >
            {isConnected ? 'Disconnect' : 'Connect'}
          </button>
        </div>
      </div>

      {/* Error Display */}
      {error && (
        <div style={styles.error}>
          <span>{error}</span>
          <button onClick={clearError}>Dismiss</button>
        </div>
      )}

      {/* Emergency Stop */}
      <div style={{ textAlign: 'center', marginBottom: '20px' }}>
        <button
          style={styles.emergencyButton}
          onClick={emergencyStop}
          disabled={!isConnected}
        >
          EMERGENCY STOP
        </button>
        {isEmergencyStopped && (
          <p style={{ color: '#dc3545', marginTop: '10px' }}>
            Emergency stop is active!
          </p>
        )}
      </div>

      {/* Task Status */}
      {taskStatus && taskStatus.task_type && (
        <div style={styles.section}>
          <h3 style={styles.sectionTitle}>Current Task</h3>
          <p>
            <strong>Type:</strong> {taskStatus.task_type} |{' '}
            <strong>Status:</strong> {taskStatus.status}
          </p>
          <div style={styles.progressBar}>
            <div
              style={{
                ...styles.progressFill,
                width: `${taskStatus.progress * 100}%`,
              }}
            />
          </div>
          <p>{(taskStatus.progress * 100).toFixed(1)}% Complete</p>
          <button
            style={{ ...styles.button, ...styles.dangerButton }}
            onClick={stopTask}
          >
            Stop Task
          </button>
        </div>
      )}

      {/* Robot Deployment */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Robot Deployment</h3>
        <div style={styles.row}>
          <label style={styles.label}>
            Robot Count
            <input
              type="number"
              min="1"
              max="20"
              value={robotCountInput}
              onChange={(e) => setRobotCountInput(parseInt(e.target.value) || 1)}
              style={styles.input}
            />
          </label>
          <label style={styles.label}>
            Spawn Pattern
            <select
              value={spawnPattern}
              onChange={(e) => setSpawnPattern(e.target.value)}
              style={styles.select}
            >
              <option value="grid">Grid</option>
              <option value="circle">Circle</option>
              <option value="line">Line</option>
            </select>
          </label>
          <button
            style={{ ...styles.button, ...styles.successButton, alignSelf: 'flex-end' }}
            onClick={() => spawnRobots(robotCountInput, spawnPattern)}
            disabled={!isConnected}
          >
            Spawn Robots
          </button>
          <button
            style={{ ...styles.button, ...styles.dangerButton, alignSelf: 'flex-end' }}
            onClick={deleteAllRobots}
            disabled={!isConnected}
          >
            Delete All Robots
          </button>
        </div>
        <p>Active Robots: {robotCount}</p>
      </div>

      {/* Task Selection */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Task Assignment</h3>

        {/* Follow-the-Leader */}
        <div style={{ marginBottom: '20px' }}>
          <h4>Follow-the-Leader</h4>
          <div style={styles.row}>
            <label style={styles.label}>
              Leader Mode
              <select
                value={leaderMode}
                onChange={(e) => setLeaderMode(e.target.value)}
                style={styles.select}
              >
                <option value="waypoint">Waypoint</option>
                <option value="manual">Manual Control</option>
                <option value="circular">Circular Path</option>
                <option value="square">Square Path</option>
                <option value="figure8">Figure-8</option>
                <option value="random">Random Exploration</option>
              </select>
            </label>
            <button
              style={{ ...styles.button, ...styles.primaryButton, alignSelf: 'flex-end' }}
              onClick={() => startFollowLeader({ leaderMode, waypoints })}
              disabled={!isConnected || robotCount < 2}
            >
              Start Follow-Leader
            </button>
          </div>

          {leaderMode === 'waypoint' && (
            <div>
              <h5>Waypoints</h5>
              {waypoints.map((wp, idx) => (
                <div key={idx} style={{ ...styles.row, alignItems: 'center' }}>
                  <span>#{idx + 1}</span>
                  <input
                    type="number"
                    placeholder="X"
                    value={wp.x}
                    onChange={(e) => updateWaypoint(idx, 'x', parseFloat(e.target.value))}
                    style={{ ...styles.input, width: '80px' }}
                  />
                  <input
                    type="number"
                    placeholder="Y"
                    value={wp.y}
                    onChange={(e) => updateWaypoint(idx, 'y', parseFloat(e.target.value))}
                    style={{ ...styles.input, width: '80px' }}
                  />
                  <button onClick={() => removeWaypoint(idx)}>Remove</button>
                </div>
              ))}
              <button onClick={addWaypoint}>Add Waypoint</button>
            </div>
          )}

          {leaderMode === 'manual' && (
            <div>
              <h5>Manual Control</h5>
              <div style={styles.row}>
                <button
                  onMouseDown={() => controlLeader(0.2, 0)}
                  onMouseUp={() => controlLeader(0, 0)}
                  onMouseLeave={() => controlLeader(0, 0)}
                >
                  Forward
                </button>
                <button
                  onMouseDown={() => controlLeader(-0.2, 0)}
                  onMouseUp={() => controlLeader(0, 0)}
                  onMouseLeave={() => controlLeader(0, 0)}
                >
                  Backward
                </button>
                <button
                  onMouseDown={() => controlLeader(0, 0.5)}
                  onMouseUp={() => controlLeader(0, 0)}
                  onMouseLeave={() => controlLeader(0, 0)}
                >
                  Turn Left
                </button>
                <button
                  onMouseDown={() => controlLeader(0, -0.5)}
                  onMouseUp={() => controlLeader(0, 0)}
                  onMouseLeave={() => controlLeader(0, 0)}
                >
                  Turn Right
                </button>
              </div>
            </div>
          )}
        </div>

        {/* Formation */}
        <div style={{ marginBottom: '20px' }}>
          <h4>Formation Control</h4>
          <div style={styles.row}>
            <label style={styles.label}>
              Formation Type
              <select
                value={formationType}
                onChange={(e) => setFormationType(e.target.value)}
                style={styles.select}
              >
                <option value="line">Line</option>
                <option value="triangle">Triangle</option>
                <option value="circle">Circle</option>
                <option value="square">Square</option>
                <option value="v">V-Formation</option>
                <option value="diamond">Diamond</option>
              </select>
            </label>
            <button
              style={{ ...styles.button, ...styles.primaryButton, alignSelf: 'flex-end' }}
              onClick={() => startFormation({ formationType })}
              disabled={!isConnected || robotCount < 2}
            >
              Start Formation
            </button>
          </div>
        </div>

        {/* Collaborative Transport */}
        <div style={{ marginBottom: '20px' }}>
          <h4>Collaborative Transport</h4>
          <div style={styles.row}>
            <label style={styles.label}>
              Target X
              <input
                type="number"
                value={targetX}
                onChange={(e) => setTargetX(parseFloat(e.target.value))}
                style={styles.input}
              />
            </label>
            <label style={styles.label}>
              Target Y
              <input
                type="number"
                value={targetY}
                onChange={(e) => setTargetY(parseFloat(e.target.value))}
                style={styles.input}
              />
            </label>
            <button
              style={{ ...styles.button, ...styles.primaryButton, alignSelf: 'flex-end' }}
              onClick={() => startTransport({ x: targetX, y: targetY })}
              disabled={!isConnected || robotCount < 2}
            >
              Start Transport
            </button>
          </div>
        </div>
      </div>

      {/* Environment */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Environment</h3>
        <div style={styles.row}>
          <label style={styles.label}>
            Obstacle Density
            <select
              value={obstacleDensity}
              onChange={(e) => setObstacleDensity(e.target.value)}
              style={styles.select}
            >
              <option value="low">Low (5 obstacles)</option>
              <option value="medium">Medium (12 obstacles)</option>
              <option value="high">High (20 obstacles)</option>
            </select>
          </label>
          <button
            style={{ ...styles.button, ...styles.successButton, alignSelf: 'flex-end' }}
            onClick={() => spawnObstacles(obstacleDensity)}
            disabled={!isConnected}
          >
            Spawn Obstacles
          </button>
        </div>
      </div>

      {/* Robot Status Grid */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Robot Status ({robots.length} robots)</h3>
        <div style={styles.robotGrid}>
          {robots.map((robot) => {
            const sensors = robot.sensors || sensorData.get(robot.id);
            const lidarRanges = sensors?.lidar?.ranges || [];
            const minRange = lidarRanges.length > 0 ? Math.min(...lidarRanges) : 0;

            return (
              <div key={robot.id} style={styles.robotCard}>
                <h4>{robot.id}</h4>
                <p><strong>Role:</strong> {robot.role}</p>
                <p>
                  <strong>Position:</strong> ({robot.position.x.toFixed(2)}, {robot.position.y.toFixed(2)})
                </p>
                <p><strong>Status:</strong> {robot.status}</p>
                <p>
                  <strong>Threat:</strong>{' '}
                  <span
                    style={{
                      color:
                        robot.threat_level > 0.7
                          ? 'red'
                          : robot.threat_level > 0.3
                          ? 'orange'
                          : 'green',
                    }}
                  >
                    {(robot.threat_level * 100).toFixed(0)}%
                  </span>
                </p>

                {/* LiDAR Mini Visualization */}
                {lidarRanges.length > 0 && (
                  <div style={{ marginTop: '10px' }}>
                    <p><strong>LiDAR:</strong> Min: {minRange.toFixed(2)}m</p>
                    <div
                      style={{
                        display: 'flex',
                        height: '30px',
                        alignItems: 'flex-end',
                        gap: '1px',
                        backgroundColor: '#eee',
                        padding: '2px',
                        borderRadius: '4px',
                      }}
                    >
                      {lidarRanges.slice(0, 36).map((range, idx) => {
                        const maxRange = sensors?.lidar?.range_max || 3.5;
                        const normalizedHeight = Math.min(range / maxRange, 1) * 100;
                        const color = range < 0.5 ? '#dc3545' : range < 1.0 ? '#ffc107' : '#28a745';
                        return (
                          <div
                            key={idx}
                            style={{
                              flex: 1,
                              height: `${normalizedHeight}%`,
                              backgroundColor: color,
                              minWidth: '2px',
                            }}
                            title={`${range.toFixed(2)}m at ${((idx * 10) - 180).toFixed(0)}°`}
                          />
                        );
                      })}
                    </div>
                    <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', color: '#666' }}>
                      <span>-180°</span>
                      <span>0°</span>
                      <span>180°</span>
                    </div>
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
};

export default SwarmControlPanel;
