const coordinate = (value) => {
    const number = Number(value);
    return Number.isFinite(number) ? number.toFixed(2) : null;
};

const MUTED_TASK_STATES = new Set(["Paused", "Cancelling", "Cancelled", "Failed"]);

const transportTaskNotice = (task) => {
    // The result stays on the task for history. Once motion stops, it should not
    // leave the impression that the robots are still searching or regrouping.
    if (MUTED_TASK_STATES.has(task?.state)) return null;

    const transport = task?.result?.transport;
    if (!transport || typeof transport !== "object") return null;

    const { discovery } = transport;
    if (
        discovery?.event === "payload_found" &&
        discovery.announced === true &&
        typeof discovery.finder === "string" &&
        discovery.finder
    ) {
        const x = coordinate(discovery.object_position?.x);
        const y = coordinate(discovery.object_position?.y);
        const location = x !== null && y !== null ? ` at (${x}, ${y})` : "";
        let nextStep = "The swarm is regrouping.";
        if (transport.phase === "PUSH") {
            nextStep = "The swarm is moving it together.";
        } else if (transport.phase === "DONE") {
            nextStep = "The swarm delivered it.";
        }
        return {
            severity: "success",
            message: `${discovery.finder} found the payload${location}. ${nextStep}`,
        };
    }

    const searchingCount = transport.searching_robot_count;
    if (transport.phase === "SEARCH" && Number.isInteger(searchingCount) && searchingCount > 0) {
        return {
            severity: "info",
            message: `${searchingCount} ${
                searchingCount === 1 ? "robot is" : "robots are"
            } searching for the payload.`,
        };
    }

    return null;
};

export default transportTaskNotice;
