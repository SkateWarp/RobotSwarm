import FuseScrollbars from '@fuse/core/FuseScrollbars';
import withReducer from 'app/store/withReducer';
import React, { useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import reducer from './store';
import { getLines, toggleQuickPanel } from './store/stateSlice';
import MachineDetails from './store/MachineDetails';
import {makeStyles} from "@mui/styles";
import {Accordion, AccordionDetails, AccordionSummary, Drawer} from "@mui/material";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';

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

function QuickPanel() {

	const dispatch = useDispatch();
	const state = useSelector(({ quickPanel }) => quickPanel.state.state);
	const lines = useSelector(({ quickPanel }) => quickPanel.state.lines);
	const alarmsPeMachine = useSelector(({ quickPanel }) => quickPanel.state.alarmsPerMachine);
	const maintenancePeMachine = useSelector(({ quickPanel }) => quickPanel.state.maintenancePerMachine);

	const classes = useStyles();


	useEffect(() => {
		// setUpSignalRConnection().then(() => {});
		// todo probablemete filtrar solo las que tengan alarmas y mantenimientos
		dispatch(getLines());
		// dispatch(getPendingMaintenances());
		// dispatch(getOngoingAlarms());
		/* init(id); */
	}, []);


	function calculateTotalAlarm() {

		let totalCount = 0;

		for (const line of Object.entries(alarmsPeMachine)) {

			const count = Object.values(line[1]).reduce((a, b) => a + b, 0);
			totalCount += count;
		}

		return totalCount;
	}


	function calculateTotalMaintenance() {

		let totalCount = 0;

		for (const line of Object.entries(maintenancePeMachine)) {
			const count = Object.values(line[1]).reduce((a, b) => a + b, 0);
			totalCount += count;
		}
		return totalCount;
	}


	return (
		<Drawer
			classes={{ paper: classes.root }}
			open={state}
			anchor="right"
			onClose={() => dispatch(toggleQuickPanel())}
		>
			<FuseScrollbars>
				<AppBar position="static" elevation={1} style={{ backgroundColor: '#AF0303' }}>
					<Toolbar className="flex w-full">
						<Typography variant="h5" className="p-24">
							Notificaciones
						</Typography>
					</Toolbar>
				</AppBar>

				{lines.length ? (
					<Accordion>
						<AccordionSummary
							expandIcon={<ExpandMoreIcon/>}
							aria-controls="panel1a-content"
							id="panel1a-header1"
						>
							<div className="flex justify-between w-full">
								<Typography className={classes.heading}>Alarmas Activas</Typography>
								<div className={clsx(classes.badge, 'item-badge')}>{calculateTotalAlarm()}</div>
							</div>
						</AccordionSummary>
						<AccordionDetails>
							<div className="flex flex-col">
								{lines &&
									lines.map(line => (
										<Accordion key={line.id}>
											<AccordionSummary
												expandIcon={<ExpandMoreIcon/>}
												aria-controls="panel1a-content"
												id="panel1a-header"
											>
												<div className="flex justify-between w-full">
													<Typography className={classes.heading}>
														{line.productName}
													</Typography>
													<div className={clsx(classes.badge, 'item-badge')}>
														{alarmsPeMachine[line.id]
															? Object.values(alarmsPeMachine[line.id]).reduce(
																	(a, b) => a + b,
																	0
															  )
															: 0}
													</div>
												</div>
											</AccordionSummary>
											<AccordionDetails>
												<MachineDetails lineId={line.id} isAlarm />
											</AccordionDetails>
										</Accordion>
									))}
							</div>
						</AccordionDetails>
					</Accordion>
				) : null}
				<Accordion>
					<AccordionSummary
						expandIcon={<ExpandMoreIcon/>}
						aria-controls="panel1a-content"
						id="panel1a-header"
					>
						<div className="flex justify-between w-full">
							<Typography className={classes.heading}>Mantenimientos Pendientes</Typography>
							<div className={clsx(classes.badge, 'item-badge')}>{calculateTotalMaintenance()}</div>
						</div>
					</AccordionSummary>
					<AccordionDetails>
						<div className="flex flex-col">
							{lines &&
								lines.map(line => (
									<Accordion key={line.id}>
										<AccordionSummary
											expandIcon={<ExpandMoreIcon/>}
											aria-controls="panel1a-content"
											id="maintenance-header"
										>
											<div className="flex justify-between w-full">
												<Typography className={classes.heading}>{line.productName}</Typography>
												<div className={clsx(classes.badge, 'item-badge')}>
													{maintenancePeMachine[line.id]
														? Object.values(maintenancePeMachine[line.id]).reduce(
																(a, b) => a + b,
																0
														  )
														: 0}
												</div>
											</div>
										</AccordionSummary>
										<AccordionDetails>
											<MachineDetails lineId={line.id} isAlarm={false} />
										</AccordionDetails>
									</Accordion>
								))}
						</div>
					</AccordionDetails>
				</Accordion>
			</FuseScrollbars>
		</Drawer>
	);
}

export default withReducer('quickPanel', reducer)(React.memo(QuickPanel));
