using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct Pose
{
    public Vector3 Position;
    public Quaternion Rotation;
}
