import { useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import { LinearProgress } from "@mui/material";
import { makeStyles } from "@mui/styles";

const useStyles2 = makeStyles({
    root: {
        width: "100%",
    },
});

const ProgressBar = ({ setOpen, callBack }) => {
    const classes2 = useStyles2();
    const [progress, setProgress] = useState(0);
    const [buffer, setBuffer] = useState(10);

    const progressRef = useRef(() => {});
    useEffect(() => {
        progressRef.current = () => {
            if (progress > 100) {
                setOpen(false);
                callBack();
            } else {
                const diff = Math.random() * 30;
                const diff2 = Math.random() * 30;
                setProgress(progress + diff);
                setBuffer(progress + diff + diff2);
            }
        };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [progress]);

    useEffect(() => {
        const timer = setInterval(() => {
            progressRef.current();
        }, 500);

        return () => {
            clearInterval(timer);
        };
    }, []);

    return (
        <div className={classes2.root}>
            <LinearProgress variant="buffer" value={progress} valueBuffer={buffer} />
        </div>
    );
};

ProgressBar.propTypes = {
    setOpen: PropTypes.func.isRequired,
    callBack: PropTypes.func.isRequired,
};

export default ProgressBar;
