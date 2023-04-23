import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import axios from 'axios';
import clsx from 'clsx';
import { useSelector } from 'react-redux';
import MaintenanceQuickPanel from '../MaintenanceQuickPanel';
import { URL } from '../../../../constants/constants';
import AlarmQuickPanel from '../AlarmQuickPanel';
import {makeStyles} from "@mui/styles";
import {Accordion, AccordionDetails, AccordionSummary} from "@mui/material";
import Typography from "@mui/material/Typography";
import List from "@mui/material/List";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";

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

function MachineDetails({ lineId, isAlarm }) {
	const [machines, setMachines] = useState([]);
	const classes = useStyles();
	const alarmsPeMachine = useSelector(({ quickPanel }) => quickPanel.state.alarmsPerMachine);
	const maintenancePeMachine = useSelector(({ quickPanel }) => quickPanel.state.maintenancePerMachine);

	useEffect(() => {
		axios.get(`${URL}/api/Machine/machine/all/${lineId}`).then(res => {
			setMachines(res.data);
		});
	}, []);

	if (isAlarm)
		return (
			<div className="flex flex-col">
				{machines &&
					machines.map(machine => (
						<Accordion key={machine.id}>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon/>}
								aria-controls="panel1a-content"
								id={`machine-${machine.id}-detail-header`}
								disabled={
									alarmsPeMachine[lineId] &&
									[machine.id] in alarmsPeMachine[lineId] &&
									alarmsPeMachine[lineId][machine.id] === 0
								}
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>{machine.model}</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{alarmsPeMachine[lineId] && [machine.id] in alarmsPeMachine[lineId]
											? alarmsPeMachine[lineId][machine.id]
											: 0}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									<AlarmQuickPanel machineId={machine.id} lineId={lineId} />
								</List>
							</AccordionDetails>
						</Accordion>
					))}
			</div>
		);

	return (
		<div className="flex flex-col">
			{machines &&
				machines.map(machine => (
					<Accordion key={machine.id}>
						<AccordionSummary
							expandIcon={<ExpandMoreIcon />}
							aria-controls="panel1a-content"
							id="maint-panel1a-header"
							disabled={
								maintenancePeMachine[lineId] &&
								[machine.id] in maintenancePeMachine[lineId] &&
								maintenancePeMachine[lineId][machine.id] === 0
							}
						>
							<div className="flex justify-between w-full">
								<Typography className={classes.heading}>{machine.model}</Typography>
								<div className={clsx(classes.badge, 'item-badge')}>
									{maintenancePeMachine[lineId] && [machine.id] in maintenancePeMachine[lineId]
										? maintenancePeMachine[lineId][machine.id]
										: 0}
								</div>
							</div>
						</AccordionSummary>
						<AccordionDetails>
							<List>
								<MaintenanceQuickPanel machineId={machine.id} lineId={lineId} />
							</List>
						</AccordionDetails>
					</Accordion>
				))}
		</div>
	);
}

export default MachineDetails;

MachineDetails.propTypes = {
	lineId: PropTypes.number.isRequired,
	isAlarm: PropTypes.bool.isRequired
};
