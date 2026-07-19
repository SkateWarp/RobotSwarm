import transportTaskNotice from "./transportTaskStatus";

describe("transport task notice", () => {
    it("shows how many robots are searching", () => {
        const notice = transportTaskNotice({
            state: "Running",
            result: {
                transport: {
                    phase: "SEARCH",
                    searching_robot_count: 10,
                    discovery: null,
                },
            },
        });

        expect(notice).toEqual({
            severity: "info",
            message: "10 robots are searching for the payload.",
        });
    });

    it("uses natural singular wording for one robot", () => {
        const notice = transportTaskNotice({
            state: "Running",
            result: {
                transport: {
                    phase: "SEARCH",
                    searching_robot_count: 1,
                    discovery: null,
                },
            },
        });

        expect(notice.message).toBe("1 robot is searching for the payload.");
    });

    it("replaces search status with the finder regrouping notice", () => {
        const notice = transportTaskNotice({
            state: "Running",
            result: {
                transport: {
                    phase: "APPROACH",
                    searching_robot_count: 0,
                    discovery: {
                        event: "payload_found",
                        announced: true,
                        finder: "tb3_4",
                        object_position: { x: 1.253, y: -0.746 },
                    },
                },
            },
        });

        expect(notice).toEqual({
            severity: "success",
            message: "tb3_4 found the payload at (1.25, -0.75). The swarm is regrouping.",
        });
    });

    it("updates the notice once the fleet starts pushing", () => {
        const notice = transportTaskNotice({
            state: "Running",
            result: {
                transport: {
                    phase: "PUSH",
                    discovery: {
                        event: "payload_found",
                        announced: true,
                        finder: "tb3_2",
                        object_position: { x: 2, y: 3 },
                    },
                },
            },
        });

        expect(notice.message).toBe(
            "tb3_2 found the payload at (2.00, 3.00). The swarm is moving it together."
        );
    });

    it("does not turn a failed task into a successful regrouping notice", () => {
        const notice = transportTaskNotice({
            state: "Failed",
            result: {
                transport: {
                    phase: "APPROACH",
                    discovery: {
                        event: "payload_found",
                        announced: true,
                        finder: "tb3_3",
                    },
                },
            },
        });

        expect(notice).toBeNull();
    });

    it.each(["Paused", "Cancelling", "Cancelled"])(
        "hides stale movement details while a task is %s",
        (state) => {
            const notice = transportTaskNotice({
                state,
                result: {
                    transport: {
                        phase: "APPROACH",
                        discovery: {
                            event: "payload_found",
                            announced: true,
                            finder: "tb3_7",
                        },
                    },
                },
            });

            expect(notice).toBeNull();
        }
    );

    it("ignores absent or incomplete transport details", () => {
        expect(transportTaskNotice({ result: null })).toBeNull();
        expect(
            transportTaskNotice({
                result: {
                    transport: {
                        phase: "APPROACH",
                        discovery: { event: "payload_found", announced: false },
                    },
                },
            })
        ).toBeNull();
    });
});
