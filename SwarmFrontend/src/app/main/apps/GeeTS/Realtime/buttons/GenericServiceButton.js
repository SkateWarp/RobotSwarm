import { ServiceCaller } from "rosreact";
import { useState, useEffect } from "react";
import Button from "@mui/material/Button";

function GenericServiceButton({ topicName, text, serviceType }) {
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
                {text}
            </Button>

            <ServiceCaller
                name={topicName}
                serviceType={ serviceType ?? "empty"}
                request={{}}
                trigger={trigger}
                callback={(resp) => {
                    console.log(resp, { topicName });
                }}
                failedCallback={(error) => {
                    console.log(error);
                }}
            />
        </>
    );
}

export default GenericServiceButton;
