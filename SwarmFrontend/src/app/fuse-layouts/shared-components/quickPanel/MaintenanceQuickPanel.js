import React, { useEffect, useState } from 'react';
import axios from 'axios';
import PropTypes from 'prop-types';
import { useDispatch } from 'react-redux';
import { URL } from '../../../constants/constants';
import singletonInstance from '../../../services/SignalRService/signalRConnectionService';
import { addMaintenanceLength, clearMaintenance } from './store/stateSlice';
import {makeStyles} from "@mui/styles";
import {ListItem, ListItemText} from "@mui/material";
import Typography from "@mui/material/Typography";

const useStyles = makeStyles(theme => ({
	root: {
		width: 280
	},
	heading: {
		fontSize: theme.typography.pxToRem(15),
		fontWeight: theme.typography.fontWeightRegular
	},
	column: {
		flexBasis: '33.33%'
	},
	badge: {
		padding: '0 10px',
		fontSize: 11,
		fontWeight: 600,
		height: 20,
		minWidth: 20,
		borderRadius: 20,
		display: 'flex',
		alignItems: 'center',
		marginLeft: '5px',
		backgroundColor: theme.palette.secondary.light,
		color: theme.palette.getContrastText(theme.palette.secondary.light)
	}
}));

function MaintenanceQuickPanel({ machineId, lineId }) {
	const [maintenances, setMaintenances] = useState([]);
	const dispatch = useDispatch();
	const classes = useStyles();

	useEffect(() => {
		dispatch(clearMaintenance({ lineId, machineId }));
		if (maintenances && maintenances.length) {
			dispatch(addMaintenanceLength({ lineId, machineId, maintenance: maintenances.length }));
		}
	}, [maintenances]);

	useEffect(() => {
		axios
			.get(`${URL}/api/MachineMaintenanceLog/percentage/machine/${machineId}`, {
				params: { percentage: 0.6 }
			})
			.then(res => {
				setMaintenances(res.data);
			});

		setUpSignalRConnection().then(() => {});
	}, []);

	const setUpSignalRConnection = async (/* realTimeId */) => {
		const connection = singletonInstance.createConnectionBuilder();

		connection.on(`PendingMaintenance/${machineId}`, message => {
			setMaintenances(message);
		});

		return connection;
	};

	return (
		<div className="">
			{maintenances &&
				maintenances.map((maintenanceInside, index) => (
					<ListItem key={index}>
						<ListItemText
							primary={`${maintenanceInside.machineMaintenance.description}`}
							secondary={
								<>
									<Typography
										component="span"
										variant="body2"
										className={classes.inline}
										color="textSecondary"
									>
										{`Días corridos: ${maintenanceInside.timeRun}`}
									</Typography>

								</>
							}
						/>
					</ListItem>
				))}
		</div>
	);
}

export default MaintenanceQuickPanel;

MaintenanceQuickPanel.propTypes = {
	machineId: PropTypes.number.isRequired
};
