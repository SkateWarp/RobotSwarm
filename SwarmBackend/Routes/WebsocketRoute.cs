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
        result = await webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);

        if (result.MessageType == WebSocketMessageType.Text)
        {
          var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
          // Log the message (optional)
          Console.WriteLine($"Received message: {message}");

          // Forward the message to SignalR clients
          await hubContext.Clients.All.SendAsync("SendCommand", message);
        }
        else if (result.MessageType == WebSocketMessageType.Close)
        {
          await webSocket.CloseAsync(result.CloseStatus.Value, result.CloseStatusDescription, CancellationToken.None);
        }
      } while (!result.CloseStatus.HasValue);
    }
  }
}