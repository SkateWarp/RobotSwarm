using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class WorkerDrainLease
{
    public static bool IsActive(ComputeWorker worker, DateTime now)
    {
        return worker.DrainLeaseId.HasValue
            && worker.DrainRequestedAt.HasValue
            && worker.DrainLeaseExpiresAt > now
            && !string.IsNullOrWhiteSpace(worker.DrainTargetRevision);
    }

    public static void ApplyHeartbeatState(ComputeWorker worker, DateTime now)
    {
        if (IsActive(worker, now))
        {
            worker.State = ComputeWorkerState.Draining;
            return;
        }

        Clear(worker);
        worker.State = ComputeWorkerState.Online;
    }

    public static void Clear(ComputeWorker worker)
    {
        worker.DrainLeaseId = null;
        worker.DrainTargetRevision = null;
        worker.DrainRequestedAt = null;
        worker.DrainLeaseExpiresAt = null;
    }
}
