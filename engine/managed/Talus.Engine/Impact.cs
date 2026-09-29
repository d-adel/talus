using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct Impact
{
    public Entity Entity;
    public Entity Other;
    public Vector3 Point;
    public Vector3 Outward;
    public float Impulse;
    public float Speed;
    public uint Material;
    public uint OtherMaterial;
}
