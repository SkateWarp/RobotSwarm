import withReducer from 'app/store/withReducer';
import { useEffect, memo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import NotificationTemplate from './NotificationTemplate';
import { addManyNotification, dismissItem, selectNotifications } from './store/dataSlice';
import reducer from './store';
import { useSnackbar } from 'notistack';
import singletonInstance from '../../../services/SignalRService/signalRConnectionService';
import { addMechanicCalls, getMachinesStop } from '../quickPanel/store/stateSlice';
import NotificationModel from './model/NotificationModel';


let displayed = [];

const storeDisplayed = id => {

	displayed = [...displayed, id];
};


function NotificationPanel() {

	const mechanicCalls = useSelector(({ quickPanel }) => quickPanel.state.mechanicCalls);
	const dispatch = useDispatch();

	const notifications = useSelector(selectNotifications);

	const { enqueueSnackbar, closeSnackbar } = useSnackbar();


	const setUpSignalRConnection = async () => {

		const connection = singletonInstance.createConnectionBuilder();

		connection.on('MechanicCall', current => {

			dispatch(addMechanicCalls(current));
		});

		return connection;
	};


	useEffect(() => {

		dispatch(getMachinesStop());

		setUpSignalRConnection().then(() => {});

	}, []);


	useEffect(() => {

		if (mechanicCalls.length > 0) {

			sendMechanicCallsNotifications();
		}

	}, [mechanicCalls]);


	function sendMechanicCallsNotifications() {

		const notifications = [];

		closeSnackbar();
		mechanicCalls.map(mechanicCall => {

			if (mechanicCall.dateMechanicResponded !== null) {

				const model = NotificationModel({

					options: { variant: 'warning' },
					message: 'Llamada al mecánico en proceso de la máquina: ' + mechanicCall.machine.model
				});

				notifications.push(model);
			} else {
				const model = NotificationModel({

					options: { variant: 'error' },
					message: 'Llamada al mecánico en proceso de la máquina: ' + mechanicCall.machine.model
				});

				notifications.push(model);
			}
		});

		dispatch(addManyNotification(notifications));
	}

	useEffect(() => {

		notifications.forEach(item => {

			const { id: key, message, options = {}, dismissed = false } = item;

			if (dismissed) {

				closeSnackbar(key);
				return;
			}

			if (displayed.includes(key)) {

				return;
			}


			enqueueSnackbar(message, {
				key,
				...options,
				autoHideDuration: null,
				content: () => (

					<NotificationTemplate
						item={item}
						onClose={() => {
							closeSnackbar(key);
							dispatch(dismissItem(key));
						}}
					/>
				),
				onClose: (event, reason, myKey) => {

					if (options.onClose) {

						options.onClose(event, reason, myKey);
					}
				},
				onExited: () => {
				}
			});

			storeDisplayed(key);
		});

	}, [notifications, closeSnackbar, enqueueSnackbar, dispatch]);


	return(
		<></>
	);

}

export default withReducer('notificationPanel', reducer)(memo(NotificationPanel));
