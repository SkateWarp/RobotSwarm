import clsx from 'clsx';
import PropTypes from 'prop-types';
import Typography from "@mui/material/Typography";
import useFragaStopTypeText from "./hooks/useFragaStopTypeText";

function FragaStopTypeLabel({status}) {

	const getStopTypeText = useFragaStopTypeText();


	function getColorText() {
		switch (status) {
			case 0:
				return 'bg-blue text-white';
			case 1:
				return 'bg-orange text-white';
			case 2:
				return 'bg-green-400 text-white';
			case 3:
				return 'bg-orange-900 text-white';
			case 4:
				return 'bg-green-900 text-white';
			case 5:
				return 'bg-red-900 text-white';
			case 6:
				return 'bg-yellow-900 text-white';
			default:
				return '';
		}
	}

	return (
		<Typography className={clsx(`${getColorText()} inline text-11 font-500 px-8 py-4 rounded-4`)}>
			{getStopTypeText(status)}
		</Typography>
	);
}

FragaStopTypeLabel.propTypes = {

	status: PropTypes.number.isRequired,
};

export default FragaStopTypeLabel;
