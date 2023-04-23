import clsx from "clsx";
import PropTypes from "prop-types";
import Typography from "@mui/material/Typography";
import useTaskCategoriesText from "./hooks/useTaskCategoriesText";

function TaskCategoriesLabel({ status }) {
    const taskCategoryText = useTaskCategoriesText(status);

    return (
        <Typography
            noWrap
            className={clsx(`bg-gray-900 text-white inline text-11 font-500 px-8 py-4 rounded-4 uppercase`)}
        >
            {taskCategoryText}
        </Typography>
    );
}

TaskCategoriesLabel.propTypes = {
    status: PropTypes.number.isRequired,
};

export default TaskCategoriesLabel;
