import React, { useEffect, useState } from 'react';
import axios from 'axios';
import PropTypes from 'prop-types';
import moment from 'moment/moment';
import { useDispatch } from 'react-redux';
import { URL } from '../../../constants/constants';
import singletonInstance from '../../../services/SignalRService/signalRConnectionService';
import { addAlarmLength, clearAlarms } from './store/stateSlice';
import {ListItem, ListItemText} from "@mui/material";

function AlarmQuickPanel({ machineId, lineId }) {
	const [alarms, setAlarms] = useState([]);
	const dispatch = useDispatch();

	useEffect(() => {
		dispatch(clearAlarms({ lineId, machineId }));
		if (alarms && alarms.length) {
			dispatch(addAlarmLength({ lineId, machineId, alarms: alarms.length }));
		}
	}, [alarms]);

	useEffect(() => {
		axios.get(`${URL}/api/Alarm/Machine/${machineId}`, { params: { limit: 10, finished: false } }).then(res => {
			setAlarms(res.data);
		});

		setUpSignalRConnection().then(() => {});
	}, []);

	const setUpSignalRConnection = async (/* realTimeId */) => {
		const connection = singletonInstance.createConnectionBuilder();

		connection.on(`CurrentAlarms/${machineId}`, message => {
			setAlarms(message);
		});

		return connection;
	};

	return (
		<div className="flex flex-col">
			{alarms &&
				alarms.map(alarmInside => (
					<ListItem key={alarmInside.id}>
						<ListItemText
							primary={alarmInside.alarmReason.description}
							secondary={moment(alarmInside.start).format('DD-MM-YYYY HH:mm')}
						/>
					</ListItem>
				))}
		</div>
	);
}

export default AlarmQuickPanel;

AlarmQuickPanel.propTypes = {
	machineId: PropTypes.number.isRequired
};
