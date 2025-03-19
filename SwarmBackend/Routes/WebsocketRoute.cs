using System.Net.WebSockets;
using System.Text;
using Microsoft.AspNetCore.SignalR;
using SwarmBackend.Services;

namespace SwarmBackend.Routes
{
  public static class WebsocketRoute
  {
    public static RouteGroupBuilder MapWebSocket(this RouteGroupBuilder group)
    {
      group.MapGet("/ws", async (HttpContext context, IHubContext<RobotHub> hubContext) =>
      {
        if (context.WebSockets.IsWebSocketRequest)
        {
          var webSocket = await context.WebSockets.AcceptWebSocketAsync();
          await HandleWebSocketConnection(webSocket, hubContext);
        }
        else
        {
          context.Response.StatusCode = 400;
        }
      });

      return group;
    }


    public static async Task HandleWebSocketConnection(WebSocket webSocket, IHubContext<RobotHub> hubContext)
    {
      var buffer = new byte[1024 * 4];
      WebSocketReceiveResult result;

      do
      {
        // Receive a message from the WebSocket
        result = await webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);

        if (result.MessageType == WebSocketMessageType.Text)
        {
          // Handle text messages
          var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
          Console.WriteLine($"Received message: {message}");

          // Forward the message to SignalR clients
          await hubContext.Clients.All.SendAsync("SendCommand", message);
        }
        else if (result.MessageType == WebSocketMessageType.Binary)
        {
          // Optionally handle binary messages (e.g., log or ignore)
          Console.WriteLine("Binary message received, ignoring...");
        }
        else if (result.MessageType == WebSocketMessageType.Close)
        {
          // Handle close messages and terminate the connection
          Console.WriteLine("WebSocket connection is closing...");
          await webSocket.CloseAsync(result.CloseStatus.Value, result.CloseStatusDescription, CancellationToken.None);
        }
        else
        {
          // Handle unexpected message types (e.g., Ping, Pong)
          Console.WriteLine($"Unexpected WebSocket message type: {result.MessageType}");
        }
      } while (!result.CloseStatus.HasValue); // Keep the connection open until a Close frame is received
    }

  }
}