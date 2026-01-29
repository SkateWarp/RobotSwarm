import { useRef } from "react";
import { VncScreen } from "react-vnc";

const VncViewer = ({ url, username, password }) => {
    const ref = useRef();

    return (
        <VncScreen
            url={url}
            rfbOptions={{
                credentials: {
                    username,
                    password,
                },
            }}
            scaleViewport
            background="#000000"
            style={{
                width: "100%",
                height: "65vh",
            }}
            ref={ref}
        />
    );
};

export default VncViewer;
