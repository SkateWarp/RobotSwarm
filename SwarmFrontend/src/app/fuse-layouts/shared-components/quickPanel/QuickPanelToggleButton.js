import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { toggleQuickPanel } from './store/stateSlice';
import IconButton from "@mui/material/IconButton";
import {Badge} from "@mui/material";
import NotificationsIcon from '@mui/icons-material/Notifications';

function QuickPanelToggleButton() {

	const dispatch = useDispatch();
	const notificationsLength = useSelector(({ quickPanel }) => quickPanel.state.totalNotifications);

	return (
		<IconButton className="w-40 h-40" onClick={() => dispatch(toggleQuickPanel())}>
			<Badge badgeContent={notificationsLength} color="primary">
				<NotificationsIcon/>
			</Badge>
		</IconButton>
	);
}

export default QuickPanelToggleButton;
