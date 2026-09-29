using System.Runtime.InteropServices;

namespace Talus.Engine;

internal enum FieldKind : uint
{
    Float = 0,
    Int = 1,
    UInt = 2,
    Bool = 3,
    Vec2 = 4,
    Vec3 = 5,
    Vec4 = 6,
    Quat = 7,
    UVec3 = 8,
    Text = 9
}


[StructLayout(LayoutKind.Sequential)]
internal unsafe struct FieldDescriptor
{
    public byte* Name;
    public uint Kind;
    public uint Offset;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct SystemContext
{
    public uint Size;
    public void* World;
    public void* Input;
    public Impact* Impacts;
    public uint ImpactCount;
    public float DeltaSeconds;
    public uint Frame;
    public void* Debug;
    public void* Queries;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct ApiTable
{
    public const uint ExpectedVersion = 16;

    public uint Size;
    public uint Version;

    public delegate* unmanaged<void*, byte*, uint, FieldDescriptor*, uint, byte**, uint, uint> DeclareComponent;
    public delegate* unmanaged<void*, byte*, uint> FindComponent;
    public delegate* unmanaged<void*, uint, uint, void*, int> SetComponent;
    public delegate* unmanaged<void*, uint, uint, void*> GetComponent;
    public delegate* unmanaged<void*, uint, uint, int> RemoveComponent;
    public delegate* unmanaged<void*, uint, uint> ComponentCount;
    public delegate* unmanaged<void*, uint, uint*> ComponentEntities;
    public delegate* unmanaged<void*, uint, void*> ComponentData;
    public delegate* unmanaged<void*, uint, SdfEditDescriptor*, SdfDeformSummary*, int> DeformSdf;
    public delegate* unmanaged<void*, uint, SdfEditDescriptor*, uint*, uint, uint> MaterialsUnderSdf;
    public delegate* unmanaged<void*, uint, float*, float> SampleSdf;
    public delegate* unmanaged<void*, uint, float*, int> GetPose;
    public delegate* unmanaged<void*, uint, float*, int> SetPose;
    public delegate* unmanaged<void*, uint*, uint, uint> Entities;
    public delegate* unmanaged<void*, uint, byte*, int> HasComponent;
    public delegate* unmanaged<void*, uint, float*, float*, int> Bounds;
    public delegate* unmanaged<byte*, void> Log;
    public delegate* unmanaged<void*, uint, int, byte*, delegate* unmanaged<void*, SystemContext*, void>, void*, int> AddSystem;
    public delegate* unmanaged<void*, uint, int> KeyHeld;
    public delegate* unmanaged<void*, uint, int> KeyPressed;
    public delegate* unmanaged<void*, uint, int> KeyReleased;
    public delegate* unmanaged<void*, uint, int> MouseHeld;
    public delegate* unmanaged<void*, uint, int> MousePressed;
    public delegate* unmanaged<void*, float*, void> Cursor;
    public delegate* unmanaged<void*, float*, uint*, float, uint> CreateSdfGrid;
    public delegate* unmanaged<void*, uint, int> Destroy;
    public delegate* unmanaged<void*, uint, SdfEditDescriptor*, int> AddPrimitive;
    public delegate* unmanaged<void*, uint, uint, int> ShareSdf;
    public delegate* unmanaged<void*, uint> MainCamera;
    public delegate* unmanaged<void*, uint, float, int> SetRigidBody;
    public delegate* unmanaged<void*, uint, float*, int> SetVelocity;
    public delegate* unmanaged<void*, uint, float*, float*, float*, int> ScreenPointToRay;
    public delegate* unmanaged<void*, float*, void> Viewport;
    public delegate* unmanaged<void*, float*, float*, float*, float, byte*, int, int> DrawLine;
    public delegate* unmanaged<void*, byte*, int, int> ShowDebug;
    public delegate* unmanaged<void*, float*, float, RaycastHit*, int> Raycast;
    public delegate* unmanaged<void*, byte*, int> RemoveSystem;
    public delegate* unmanaged<void*, int> Restart;
    public delegate* unmanaged<void*, uint, PhysicalMaterial*, int> Material;
}

public enum Shape : uint
{
    Sphere = 0,
    Box = 1,
    Plane = 2,
    Cylinder = 3
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct SdfEditDescriptor
{
    public uint Kind;
    public uint Material;
    public uint Operation;
    public float Blend;
    public fixed float Position[3];
    public fixed float Size[3];
    public fixed float Rotation[4];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct SdfDeformSummary
{
    public uint CellsRemoved;
    public float VolumeRemoved;
    public fixed uint BrickBegin[3];
    public fixed uint BrickEnd[3];
    public fixed uint SampleMinimum[3];
    public fixed uint SampleExtent[3];
}
