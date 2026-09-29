namespace Talus.Engine;

public enum Operation : uint
{
    Union = 0,
    SmoothUnion = 1,
    Intersect = 2,
    Subtract = 3,
    SmoothSubtract = 4
}

public readonly record struct DeformResult(uint CellsRemoved, float VolumeRemoved);

public struct SdfEdit
{
    public Shape Shape;
    public Operation Operation;
    public Vector3 Position;
    public Vector3 Size;
    public Quaternion Rotation;
    public uint Material;
    public float Blend;

    public static SdfEdit Sphere(Vector3 at, float radius) => new()
    {
        Shape = Shape.Sphere,
        Position = at,
        Size = new Vector3(radius, radius, radius),
        Rotation = Quaternion.Identity
    };

    public static SdfEdit Box(Vector3 at, Vector3 half) => new()
    {
        Shape = Shape.Box,
        Position = at,
        Size = half,
        Rotation = Quaternion.Identity
    };

    public static SdfEdit Plane(Vector3 at, Quaternion facing) => new()
    {
        Shape = Shape.Plane,
        Position = at,
        Rotation = facing
    };

    public static SdfEdit Cylinder(Vector3 at, float radius, float halfLength)
        => new()
        {
            Shape = Shape.Cylinder,
            Position = at,
            Size = new Vector3(radius, halfLength, radius),
            Rotation = Quaternion.Identity
        };

    public SdfEdit Subtracting() => With(Operation.Subtract);

    public SdfEdit Adding(uint material) => With(Operation.Union, material);

    public SdfEdit With(Operation operation, uint material = 1u)
    {
        SdfEdit edit = this;

        edit.Operation = operation;
        edit.Material = material;

        return edit;
    }

    internal unsafe SdfEditDescriptor ToDescriptor()
    {
        SdfEditDescriptor descriptor = default;

        descriptor.Kind = (uint)Shape;
        descriptor.Material = Material;
        descriptor.Operation = (uint)Operation;
        descriptor.Blend = Blend;

        descriptor.Position[0] = Position.X;
        descriptor.Position[1] = Position.Y;
        descriptor.Position[2] = Position.Z;

        descriptor.Size[0] = Size.X;
        descriptor.Size[1] = Size.Y;
        descriptor.Size[2] = Size.Z;

        descriptor.Rotation[0] = Rotation.X;
        descriptor.Rotation[1] = Rotation.Y;
        descriptor.Rotation[2] = Rotation.Z;
        descriptor.Rotation[3] = Rotation.W;

        return descriptor;
    }
}
