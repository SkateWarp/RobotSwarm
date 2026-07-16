namespace SwarmBackend.Entities;

public enum ComputeWorkerState
{
    Offline = 0,
    Online = 1,
    Draining = 2,
    Unhealthy = 3
}

// Values before Stopped are deliberately non-terminal. The database uses this
// ordering for the one-live-session-per-account partial unique index.
public enum SimulationSessionState
{
    Queued = 0,
    Provisioning = 1,
    Ready = 2,
    Active = 3,
    Paused = 4,
    Stopping = 5,
    Stopped = 6,
    Failed = 7,
    Expired = 8
}

public enum SessionRobotState
{
    Provisioning = 0,
    Ready = 1,
    Active = 2,
    Offline = 3,
    Removed = 4,
    Failed = 5
}

public enum SwarmTaskRunType
{
    FollowLeader = 0,
    Figure = 1,
    CollaborativeTransport = 2
}

public enum TaskRunState
{
    Queued = 0,
    Accepted = 1,
    Running = 2,
    Paused = 3,
    Cancelling = 4,
    Completed = 5,
    Cancelled = 6,
    Failed = 7
}

public enum WorkerCommandType
{
    ProvisionSession = 0,
    UpdateFleet = 1,
    StartTask = 2,
    PauseTask = 3,
    ResumeTask = 4,
    CancelTask = 5,
    EmergencyStop = 6,
    ResetEmergencyStop = 7,
    StopSession = 8,
    SetViewerSource = 9
}

public enum WorkerCommandState
{
    Pending = 0,
    Dispatched = 1,
    Acknowledged = 2,
    Completed = 3,
    Failed = 4,
    Cancelled = 5,
    Running = 6
}

public enum ViewerSourceType
{
    Scene = 0,
    RobotCamera = 1
}
