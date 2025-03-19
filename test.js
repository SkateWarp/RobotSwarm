const socket = new WebSocket("wss://robot.zerav.la/WebSocket/ws");

socket.onopen = () => {
  console.log("WebSocket connection established");
  socket.send("Hello from WebSocket!");
};

socket.onmessage = (event) => {
  console.log("Message from server:", event.data);
};

socket.onclose = () => {
  console.log("WebSocket connection closed");
};
