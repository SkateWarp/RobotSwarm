import rospy
from signalrcore.hub_connection_builder import HubConnectionBuilder
from datetime import datetime
import json
import logging
import threading
import time
from queue import Queue
import traceback

class SignalRHandler:
    def __init__(self, backend_url, robot_id, command_callback):
        """
        Initialize SignalR handler
        
        Args:
            backend_url (str): URL of the SignalR hub
            robot_id (int): ID of the robot
            command_callback (callable): Callback for handling commands from backend
        """
        self.url = backend_url
        self.robot_id = robot_id
        self.command_callback = command_callback
        self.connection = None
        self.is_connected = False
        self.reconnect_attempt = 0
        self.max_reconnect_delay = 30
        
        # Message queue for handling connection issues
        self.message_queue = Queue()
        
        # Setup logging
        self.logger = logging.getLogger(f'signalr_handler_{robot_id}')
        self.logger.setLevel(logging.DEBUG)
        
        # Initialize connection
        self._setup_connection()
        
        # Start message processor
        self.processor_thread = threading.Thread(target=self._process_message_queue)
        self.processor_thread.daemon = True
        self.processor_thread.start()

    def _setup_connection(self):
        try:
            self.connection = (HubConnectionBuilder()
                .with_url(f"{self.url}?robotId={self.robot_id}")
                .with_automatic_reconnect({
                    "type": "raw",
                    "keep_alive_interval": 10,
                    "reconnect_interval": 5,
                    "max_attempts": 5
                })
                .configure_logging(logging.DEBUG)
                .build())
                
            # Add connection lifecycle handlers
            self.connection.on_open(lambda: self._handle_connect())  # Changed this line
            self.connection.on_close(lambda: self._handle_disconnect())  # Changed this line
            self.connection.on_error(lambda error: self._handle_error(error))  # Changed this line
            
            rospy.loginfo("SignalR connection setup complete")

        except Exception as e:
            self.logger.error(f"Error setting up SignalR connection: {e}")
            raise

    def _handle_connect(self):
        """Handle successful connection"""
        self.is_connected = True
        self.reconnect_attempt = 0
        self.logger.info("Connected to SignalR hub")
        
        # Send initial status
        self.send_status_update("Idle")
        
        # Process any queued messages
        self._process_queued_messages()

    def _handle_disconnect(self):
        """Handle disconnection"""
        self.is_connected = False
        self.logger.warning("Disconnected from SignalR hub")
        
        # Attempt reconnection
        self._attempt_reconnect()

    def _handle_error(self, error):
        """Handle connection errors"""
        self.logger.error(f"SignalR error: {error}")
        if self.is_connected:
            self.is_connected = False
            self._attempt_reconnect()

    def _attempt_reconnect(self):
        """Attempt to reconnect with exponential backoff"""
        if not self.is_connected:
            self.reconnect_attempt += 1
            delay = min(2 ** (self.reconnect_attempt - 1), self.max_reconnect_delay)
            
            self.logger.info(f"Attempting reconnection in {delay} seconds...")
            threading.Timer(delay, self._reconnect).start()

    def _reconnect(self):
        """Reconnect to SignalR hub"""
        try:
            if not self.is_connected:
                self.connection.start()
        except Exception as e:
            self.logger.error(f"Reconnection failed: {e}")
            self._attempt_reconnect()

    def _handle_command(self, command):
        """Handle incoming commands from SignalR"""
        try:
            self.logger.debug(f"Received command: {command}")
            if isinstance(command, str):
                command = json.loads(command)
            
            # Pass command to callback
            self.command_callback(command)
            
            # Send acknowledgment
            self.send_message("CommandAcknowledgment", {
                "commandId": command.get("id"),
                "status": "received",
                "timestamp": datetime.utcnow().isoformat()
            })
            
        except Exception as e:
            self.logger.error(f"Error handling command: {e}")
            self.send_message("CommandError", {
                "commandId": command.get("id"),
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat()
            })

    def start(self):
        """Start the SignalR connection"""
        try:
            rospy.loginfo("Starting SignalR connection...")
            self.connection.start()
            
            # Wait for connection to be established
            retry_count = 0
            while not self.is_connected and retry_count < 10:
                rospy.sleep(1)  # Wait 1 second
                retry_count += 1
                rospy.loginfo(f"Waiting for connection... Attempt {retry_count}")
                
            if self.is_connected:
                rospy.loginfo("SignalR connection established successfully")
            else:
                rospy.logerr("Failed to establish SignalR connection after 10 seconds")
                
        except Exception as e:
            rospy.logerr(f"Error starting SignalR connection: {e}")
            self._attempt_reconnect()

    def stop(self):
        """Stop the SignalR connection"""
        try:
            if self.is_connected:
                self.connection.stop()
        except Exception as e:
            self.logger.error(f"Error stopping SignalR connection: {e}")

    def send_message(self, method, data):
        """
        Send message to SignalR hub
        
        Args:
            method (str): Hub method to call
            data (dict): Data to send
        """
        try:
            if self.is_connected:
                rospy.loginfo(f"Sending message to SignalR: {method} - {data}")
                self.connection.send(method, [data])
                rospy.loginfo("Message sent successfully")
            else:
                rospy.logwarn("Not connected to SignalR, queueing message")
                # Queue message for later
                self.message_queue.put((method, data))
                self.logger.debug(f"Message queued: {method}")
        except Exception as e:
            rospy.logerr(f"Error sending message: {e}")
            rospy.logerr(f"Stack trace: {traceback.format_exc()}")            
            # Queue message for retry
            self.message_queue.put((method, data))

    def send_status_update(self, robot_id, status):
        """
        Send status update to backend
        """
        rospy.loginfo(f"Robot {robot_id} status changed to {status} from signalr")
        
        # Try sending just the status as a single argument
        try:
            rospy.loginfo(f"Attempting to send status update with robot_id={robot_id}, status={status}")
            self.connection.send("UpdateStatus", [robot_id, status])  # Try this format first
        except Exception as e:
            rospy.logerr(f"Error with direct parameters: {e}")
            # Fall back to object format
            self.send_message("UpdateStatus", {
                "robotId": robot_id,
                "status": status
            })

    def send_sensor_reading(self, robot_id, sensor_data):
        """
        Send sensor reading to backend
        
        Args:
            sensor_data (dict): Sensor reading data
        """
        rospy.loginfo(f"Robot {robot_id} sensor reading received from signalr")
        rospy.loginfo(f"Sensor data: {sensor_data}")
        # Format data to match SensorReadingRequest record
        reading_request = {
            "value": float(sensor_data.get("value")),  # double Value
            "sensorName": str(sensor_data.get("sensorName")),  # int SensorId
        }
        
        # Send to SignalR hub with both robotId and reading
        # Note: Parameters must be in same order as C# method signature
        self.connection.send("HandleSensorReading", [
            int(robot_id),  # First parameter as integer
            reading_request  # Second parameter as object
        ])

    def send_task(self, robot_id, task_data):
        """
        Send task log to backend
        
        Args:
            robot_id (int): ID of the robot
            task_data (dict): Task log data
        """
        rospy.loginfo(f"Robot {robot_id} task log received from signalr")
        rospy.loginfo(f"Task data: {task_data}")
        # Format data to match TaskLogRequest record
        task_request = {
            "taskType": str(task_data.get("taskType")),  # string TaskType
            "parameters": json.dumps(task_data.get("parameters")),  # JsonElement Parameters
        }
        
        # Send to SignalR hub with both robotId and task
        # Note: Parameters must be in same order as C# method signature
        self.connection.send("HandleTaskLog", [
            int(robot_id),  # First parameter as integer
            task_request  # Second parameter as object
        ])
        
        
    def send_finish_task(self, robot_id):
        """
        Send finish task log to backend
        
        Args:
            robot_id (int): ID of the robot
        """
        rospy.loginfo(f"Robot {robot_id} finish task log received from signalr")
        self.connection.send("HandleFinishTaskLog", [
            int(robot_id),  # First parameter as integer
        ])

    def send_cancel_task(self, robot_id):
        """
        Send cancel task log to backend
        
        Args:
            robot_id (int): ID of the robot
        """
        rospy.loginfo(f"Robot {robot_id} cancel task log received from signalr")
        self.connection.send("HandleCancelTaskLog", [
            int(robot_id),  # First parameter as integer
        ])
        
        

    def _process_message_queue(self):
        """Process queued messages when connection is restored"""
        while True:
            if self.is_connected and not self.message_queue.empty():
                try:
                    method, data = self.message_queue.get()
                    self.connection.send(method, [data])
                    self.logger.debug(f"Processed queued message: {method}")
                except Exception as e:
                    self.logger.error(f"Error processing queued message: {e}")
                    # Re-queue the message
                    self.message_queue.put((method, data))
            time.sleep(0.1)

    def _process_queued_messages(self):
        """Process all queued messages"""
        while not self.message_queue.empty():
            try:
                method, data = self.message_queue.get_nowait()
                self.connection.send(method, [data])
                self.logger.debug(f"Processed queued message: {method}")
            except Exception as e:
                self.logger.error(f"Error processing queued message: {e}")
                break

    def send_heartbeat(self):
        """Send heartbeat to backend"""
        self.send_message("Heartbeat", {
            "robotId": self.robot_id,
            "timestamp": datetime.utcnow().isoformat()
        })