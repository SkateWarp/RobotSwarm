import FuseScrollbars from '@fuse/core/FuseScrollbars';
import withReducer from 'app/store/withReducer';
import React, { useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import * as moment from 'moment';
import {
	getOngoingAlarms,
	getPendingMaintenances,
	setOngoingAlarms,
	setPendingMaintenances,
	toggleQuickPanelFraga
} from './store/stateSlice';
import reducer from './store';
import singletonInstance from '../../../services/SignalRService/signalRConnectionService';
import {makeStyles} from "@mui/styles";
import {Accordion, AccordionDetails, Drawer, ListItem, ListItemText} from "@mui/material";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import AccordionSummary from "@mui/material/AccordionSummary";
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import List from "@mui/material/List";

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

function QuickPanelFraga() {
	const dispatch = useDispatch();
	const state = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.state);
	const pendingSlitterMaintenance = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.slitterMaintenances);
	const pendingTubeMillMaintenance = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.tubeMillMaintenances);
	const pendingTolaMaintenance = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.tolaMaintenances);
	const pendingAluzincMaintenance = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.aluzincMaintenances);

	const ongoingSlitterAlarms = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.slitterAlarms);
	const ongoingTubeMillAlarms = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.tubeMillAlarms);
	const ongoingTolaAlarms = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.tolaAlarms);
	const ongoingAluzincAlarms = useSelector(({ quickPanelFraga }) => quickPanelFraga.state.aluzincAlarms);

	const classes = useStyles();
	useEffect(() => {
		const groupNames = ['CurrentAlarms/1', 'CurrentAlarms/2', 'CurrentAlarms/4', 'CurrentAlarms/5',
			'PendingMaintenance/1', 'PendingMaintenance/2', 'PendingMaintenance/4', 'PendingMaintenance/5'].join(",");

		const connection = singletonInstance.createConnectionBuilder2(groupNames);

		setUpSignalRConnection(connection).then(() => {});
		dispatch(getPendingMaintenances(1));
		dispatch(getPendingMaintenances(2));
		dispatch(getPendingMaintenances(4));
		dispatch(getPendingMaintenances(5));
		dispatch(getOngoingAlarms(1));
		dispatch(getOngoingAlarms(2));
		dispatch(getOngoingAlarms(4));
		dispatch(getOngoingAlarms(5));
		/* init(id); */
	}, []);

	const setUpSignalRConnection = async (connection) => {

		connection.on('CurrentAlarms/1', message => {
			dispatch(setOngoingAlarms({ alarms: message, machineId: 1 }));
		});

		connection.on('CurrentAlarms/2', message => {
			dispatch(setOngoingAlarms({ alarms: message, machineId: 2 }));
		});

		connection.on('CurrentAlarms/4', message => {
			dispatch(setOngoingAlarms({ alarms: message, machineId: 4 }));
		});

		connection.on('CurrentAlarms/5', message => {
			dispatch(setOngoingAlarms({ alarms: message, machineId: 5 }));
		});

		connection.on('PendingMaintenance/1', message => {
			dispatch(setPendingMaintenances({ maintenance: message, machineId: 1 }));
		});

		connection.on('PendingMaintenance/2', message => {
			dispatch(setPendingMaintenances({ maintenance: message, machineId: 2 }));
		});

		connection.on('PendingMaintenance/4', message => {
			dispatch(setPendingMaintenances({ maintenance: message, machineId: 4 }));
		});

		connection.on('PendingMaintenance/5', message => {
			dispatch(setPendingMaintenances({ maintenance: message, machineId: 5 }));
		});

		/* if (connection.state === HubConnectionState.Connected) {
			connection.invoke('SubscribeToRealTime', realTimeId).catch((err) => {
				return console.error(err.toString());
			});
		} */
	};

	return (
		<Drawer
			classes={{ paper: classes.root }}
			open={state}
			anchor="right"
			onClose={() => dispatch(toggleQuickPanelFraga())}
		>
			<FuseScrollbars>
				<AppBar position="static" elevation={1} style={{ backgroundColor: '#008daa' }}>
					<Toolbar className="flex w-full">
						<Typography variant="h5" className="p-24">
							Notificaciones
						</Typography>
					</Toolbar>
				</AppBar>

				<Accordion>
					<AccordionSummary
						expandIcon={<ExpandMoreIcon />}
						aria-controls="panel1a-content"
						id="panel1a-header"
					>
						<div className="flex justify-between w-full">
							<Typography className={classes.heading}>Slitter</Typography>
							<div className={clsx(classes.badge, 'item-badge')}>
								{ongoingSlitterAlarms.length + pendingSlitterMaintenance.length}
							</div>
						</div>
					</AccordionSummary>
					<AccordionDetails className="flex flex-col">
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Alarmas Activas</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{ongoingSlitterAlarms.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{ongoingSlitterAlarms &&
										ongoingSlitterAlarms.map(alarm => (
											<ListItem key={alarm.id}>
												<ListItemText
													primary={alarm.alarmReason && alarm.alarmReason.description}
													secondary={moment(alarm.start).format('DD-MM-YYYY')}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Mantenimientos</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{pendingSlitterMaintenance.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{pendingSlitterMaintenance &&
										pendingSlitterMaintenance.map(machineMaintenanceLog => (
											<ListItem key={machineMaintenanceLog.id}>
												<ListItemText
													secondary={
														<>
															<Typography
																component="span"
																variant="body2"
																className={classes.inline}
																color="textPrimary"
															>
																{machineMaintenanceLog &&
																	`Horas corridas: ${machineMaintenanceLog.hoursRun}`}
															</Typography>
															<br />
															<Typography variant="body1" component="span">
																{`Horas vida útil: ${machineMaintenanceLog.machineMaintenance.amount}`}
															</Typography>
														</>
													}
													primary={machineMaintenanceLog.machineMaintenance.procedure}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
					</AccordionDetails>
				</Accordion>
				<Accordion>
					<AccordionSummary
						expandIcon={<ExpandMoreIcon />}
						aria-controls="panel1a-content"
						id="panel1a-header"
					>
						<div className="flex justify-between w-full">
							<Typography className={classes.heading}>TubeMill</Typography>
							<div className={clsx(classes.badge, 'item-badge')}>
								{ongoingTubeMillAlarms.length + pendingTubeMillMaintenance.length}
							</div>
						</div>
					</AccordionSummary>
					<AccordionDetails className="flex flex-col">
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Alarmas Activas</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{ongoingTubeMillAlarms.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{ongoingTubeMillAlarms &&
										ongoingTubeMillAlarms.map(alarm => (
											<ListItem key={alarm.id}>
												<ListItemText
													primary={alarm.alarmReason && alarm.alarmReason.description}
													secondary={moment(alarm.start).format('DD-MM-YYYY')}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Mantenimientos</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{pendingTubeMillMaintenance.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{pendingTubeMillMaintenance &&
										pendingTubeMillMaintenance.map(machineMaintenanceLog => (
											<ListItem key={machineMaintenanceLog.id}>
												<ListItemText
													secondary={
														<>
															<Typography
																component="span"
																variant="body2"
																className={classes.inline}
																color="textPrimary"
															>
																{machineMaintenanceLog &&
																	`Horas corridas: ${machineMaintenanceLog.hoursRun}`}
															</Typography>
															<br />
															<Typography variant="body1" component="span">
																{`Horas vida útil: ${machineMaintenanceLog.machineMaintenance.amount}`}
															</Typography>
														</>
													}
													primary={machineMaintenanceLog.machineMaintenance.procedure}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
					</AccordionDetails>
				</Accordion>
				<Accordion>
					<AccordionSummary
						expandIcon={<ExpandMoreIcon />}
						aria-controls="panel1a-content"
						id="panel1a-header"
					>
						<div className="flex justify-between w-full">
							<Typography className={classes.heading}>Aluzinc</Typography>
							<div className={clsx(classes.badge, 'item-badge')}>
								{ongoingAluzincAlarms.length + pendingAluzincMaintenance.length}
							</div>
						</div>
					</AccordionSummary>
					<AccordionDetails className="flex flex-col">
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Alarmas Activas</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{ongoingAluzincAlarms.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{ongoingAluzincAlarms &&
										ongoingAluzincAlarms.map(alarm => (
											<ListItem key={alarm.id}>
												<ListItemText
													primary={alarm.alarmReason && alarm.alarmReason.description}
													secondary={moment(alarm.start).format('DD-MM-YYYY')}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Mantenimientos</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{pendingAluzincMaintenance.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{pendingAluzincMaintenance &&
										pendingAluzincMaintenance.map(machineMaintenanceLog => (
											<ListItem key={machineMaintenanceLog.id}>
												<ListItemText
													secondary={
														<>
															<Typography
																component="span"
																variant="body2"
																className={classes.inline}
																color="textPrimary"
															>
																{machineMaintenanceLog &&
																	`Horas corridas: ${machineMaintenanceLog.hoursRun}`}
															</Typography>
															<br />
															<Typography variant="body1" component="span">
																{`Horas vida útil: ${machineMaintenanceLog.machineMaintenance.amount}`}
															</Typography>
														</>
													}
													primary={machineMaintenanceLog.machineMaintenance.procedure}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
					</AccordionDetails>
				</Accordion>
				<Accordion>
					<AccordionSummary
						expandIcon={<ExpandMoreIcon />}
						aria-controls="panel1a-content"
						id="panel1a-header"
					>
						<div className="flex justify-between w-full">
							<Typography className={classes.heading}>Tola</Typography>
							<div className={clsx(classes.badge, 'item-badge')}>
								{ongoingTolaAlarms.length + pendingTolaMaintenance.length}
							</div>
						</div>
					</AccordionSummary>
					<AccordionDetails className="flex flex-col">
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Alarmas Activas</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{ongoingTolaAlarms.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{ongoingTolaAlarms &&
										ongoingTolaAlarms.map(alarm => (
											<ListItem key={alarm.id}>
												<ListItemText
													primary={alarm.alarmReason && alarm.alarmReason.description}
													secondary={moment(alarm.start).format('DD-MM-YYYY')}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
						<Accordion>
							<AccordionSummary
								expandIcon={<ExpandMoreIcon />}
								aria-controls="panel1a-content"
								id="panel1a-header"
							>
								<div className="flex justify-between w-full">
									<Typography className={classes.heading}>Mantenimientos</Typography>
									<div className={clsx(classes.badge, 'item-badge')}>
										{pendingTolaMaintenance.length}
									</div>
								</div>
							</AccordionSummary>
							<AccordionDetails>
								<List>
									{/* <ListSubheader component="div"></ListSubheader> */}
									{pendingTolaMaintenance &&
										pendingTolaMaintenance.map(machineMaintenanceLog => (
											<ListItem key={machineMaintenanceLog.id}>
												<ListItemText
													secondary={
														<>
															<Typography
																component="span"
																variant="body2"
																className={classes.inline}
																color="textPrimary"
															>
																{machineMaintenanceLog &&
																	`Horas corridas: ${machineMaintenanceLog.hoursRun}`}
															</Typography>
															<br />
															<Typography variant="body1" component="span">
																{`Horas vida útil: ${machineMaintenanceLog.machineMaintenance.amount}`}
															</Typography>
														</>
													}
													primary={machineMaintenanceLog.machineMaintenance.procedure}
												/>
											</ListItem>
										))}
								</List>
							</AccordionDetails>
						</Accordion>
					</AccordionDetails>
				</Accordion>
			</FuseScrollbars>
		</Drawer>
	);
}

export default withReducer('quickPanelFraga', reducer)(React.memo(QuickPanelFraga));
