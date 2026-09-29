using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct PhysicalMaterial
{
    public float Friction;
    public float Restitution;
    public float Density;
    public float Strength;

    public bool Unbreakable => float.IsPositiveInfinity(Strength);
}
