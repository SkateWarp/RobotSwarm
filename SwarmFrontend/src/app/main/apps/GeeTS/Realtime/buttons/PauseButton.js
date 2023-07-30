import { ServiceCaller } from "rosreact";
import { useState, useEffect } from "react";
import ToggleButton from "@mui/material/ToggleButton";
import ToggleButtonGroup from "@mui/material/ToggleButtonGroup";

function PauseButton() {
    const [current, setCurrent] = useState(0);
    const [trigger, setTrigger] = useState(false);

    useEffect(() => {
        setTrigger(true);
    }, [trigger]);

    const handleChange = (event, value) => {
        if (current !== value) setTrigger(false);
    };

    const serviceName = current === 0 ? "/gazebo/pause_physics" : "/gazebo/unpause_physics";
    return (
        <>
            <ToggleButtonGroup value={current} exclusive onChange={handleChange} aria-label="text alignment">
                <ToggleButton
                    value={0}
                    onClick={() => setCurrent(0)}
                    variant="contained"
                    color="secondary"
                    className="w-full"
                >
                    Pausar
                </ToggleButton>{" "}
                <ToggleButton
                    value={1}
                    onClick={() => setCurrent(1)}
                    variant="contained"
                    color="secondary"
                    className="w-full"
                >
                    Play
                </ToggleButton>
            </ToggleButtonGroup>

            <ServiceCaller
                name={serviceName}
                serviceType="empty"
                request={{}}
                trigger={trigger}
                callback={(resp) => {
                    console.log(resp, serviceName);
                }}
                failedCallback={(error) => {
                    console.log(error);
                }}
            />
        </>
    );
}

export default PauseButton;
