using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RosbridgeNet.RosbridgeClient.Common.Attributes;

namespace ros_wss;


[RosMessageType("geometry_msgs/Twist")]
public class Twist
{
    public Vector linear { get; set; }
    public Vector angular { get; set; }

    public override string ToString()
    {
        return string.Format("linear: {0}, angular: {1}", linear.ToString(), angular.ToString());
    }
}

public class Vector
{
    public float x { get; set; }
    public float y { get; set; }
    public float z { get; set; }

    public override string ToString()
    {
        return string.Format("x: {0}, y: {1}, z: {2}", x, y, z);
    }
}
