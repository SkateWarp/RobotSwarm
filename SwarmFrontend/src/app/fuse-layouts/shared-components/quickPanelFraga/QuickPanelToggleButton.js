import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { toggleQuickPanelFraga } from './store/stateSlice';
import IconButton from "@mui/material/IconButton";
import {Badge} from "@mui/material";
import NotificationsIcon from '@mui/icons-material/Notifications';

function QuickPanelToggleButtonFraga() {
	const dispatch = useDispatch();
	const notificationsLength = useSelector(
		({ quickPanelFraga }) =>
			quickPanelFraga.state.slitterAlarms.length +
			quickPanelFraga.state.slitterMaintenances.length +
			quickPanelFraga.state.tubeMillAlarms.length +
			quickPanelFraga.state.tubeMillMaintenances.length
	);

	return (
		<IconButton className="w-40 h-40" onClick={() => dispatch(toggleQuickPanelFraga())}>
			<Badge badgeContent={notificationsLength} color="secondary">
				<NotificationsIcon />
			</Badge>
		</IconButton>
	);
}

export default QuickPanelToggleButtonFraga;
