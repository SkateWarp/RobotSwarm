// See https://aka.ms/new-console-template for more information
using ros_wss;
using RosbridgeNet.RosbridgeClient.Common;
using RosbridgeNet.RosbridgeClient.Common.Interfaces;
using RosbridgeNet.RosbridgeClient.ProtocolV2;
using RosbridgeNet.RosbridgeClient.ProtocolV2.Generics;
using RosbridgeNet.RosbridgeClient.ProtocolV2.Generics.Interfaces;
using System.Net.WebSockets;

Console.WriteLine("Hello, World!");


string webSocketUri = "ws://192.168.0.34:9090";
CancellationTokenSource cts = new CancellationTokenSource();

IRosbridgeMessageDispatcher messageDispatcher = Connect(new Uri(webSocketUri), cts);

Console.WriteLine("Subscribiendo ...");

await Subscribe(messageDispatcher);
await Task.Delay(3000);
Console.WriteLine("Subscrito!");
Console.WriteLine("");


var publisher = await CreatePublisher(messageDispatcher);

IRosServiceClient<object, object> serviceClient = new RosServiceClient<object, object>(messageDispatcher, "/clear");

Console.WriteLine("Enviando topico");

await publisher.PublishAsync(new Twist()
{
    linear = new Vector()
    {
        x = -5,
        y = 0,
        z = 0
    },
    angular = new Vector()
    {
        x = 0,
        y = 0,
        z = 0
    }
});

//serviceClient.CallServiceAsync().Wait();

while (true)
{
    Thread.Sleep(3000);
}

static async Task<RosPublisher<Twist>> CreatePublisher(IRosbridgeMessageDispatcher messageDispatcher)
{
    RosPublisher<Twist> publisher = new RosPublisher<Twist>(messageDispatcher, "/turtle1/cmd_vel");
    await publisher.AdvertiseAsync();

    return publisher;
}

static async Task Subscribe(IRosbridgeMessageDispatcher messageDispatcher)
{
    RosSubscriber<Twist> subscriber = new(messageDispatcher, "/turtle1/cmd_vel");

    subscriber.RosMessageReceived += (s, e) => {
        Console.WriteLine("Recibiendo topico");
        Console.WriteLine(e.RosMessage); };

    await subscriber.SubscribeAsync();
}

static IRosbridgeMessageDispatcher Connect(Uri webSocketAddress, CancellationTokenSource cancellationTokenSource)
{
    ISocket socket = new Socket(new ClientWebSocket(), webSocketAddress, cancellationTokenSource);
    IRosbridgeMessageSerializer messageSerializer = new RosbridgeMessageSerializer();
    IRosbridgeMessageDispatcher messageDispatcher = new RosbridgeMessageDispatcher(socket, messageSerializer);

    messageDispatcher.StartAsync().Wait();

    return messageDispatcher;
}

public class Spawn
{
    public float x { get; set; }
    public float y { get; set; }
    public float theta { get; set; }
    public string name { get; set; }
}

public class SpawnResponse
{
    public string name { get; set; }
}