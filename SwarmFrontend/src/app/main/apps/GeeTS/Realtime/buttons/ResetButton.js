import { ServiceCaller } from "rosreact";
import { useState, useEffect } from "react";
import Button from "@mui/material/Button";

function ResetButton() {
    const [trigger, setTrigger] = useState(false);

    useEffect(() => {
        if (trigger) setTrigger(false);
    }, [trigger]);

    const handleChange = (e) => {
        setTrigger(true);
        console.log("cambiando trigger", trigger);
    };

    return (
        <>
            <Button onClick={handleChange} aria-label="text alignment">
                Reset
            </Button>

            <ServiceCaller
                name="/gazebo/reset_simulation"
                serviceType="empty"
                request={{}}
                trigger={trigger}
                callback={(resp) => {
                    console.log(resp, "/gazebo/reset_simulation");
                }}
                failedCallback={(error) => {
                    console.log(error);
                }}
            />
        </>
    );
}

export default ResetButton;
