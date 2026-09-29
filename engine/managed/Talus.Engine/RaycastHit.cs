using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct RaycastHit
{
    public Entity Entity;
    public Vector3 Position;
    public Vector3 Normal;
    public float Distance;
    public uint Material;
}