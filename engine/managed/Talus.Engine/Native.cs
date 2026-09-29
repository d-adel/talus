using System.Runtime.InteropServices;
using System.Text;

namespace Talus.Engine;

internal static unsafe class Native
{
    public const uint InvalidComponent = uint.MaxValue;

    private static ApiTable s_api;
    private static bool s_ready;

    public static string? Refusal { get; private set; }

    private const int ImpactRecordSize = 48;

    public static bool Bind(ApiTable* api)
    {
        Refusal = null;

        if (api == null)
        {
            Refusal = "the engine passed no api table";

            return false;
        }

        if (api->Version != ApiTable.ExpectedVersion)
        {
            Refusal =
                $"api version {api->Version}, expected {ApiTable.ExpectedVersion}";

            return false;
        }

        if (api->Size != (uint)sizeof(ApiTable))
        {
            Refusal = $"api table is {api->Size} bytes, expected {sizeof(ApiTable)}";

            return false;
        }

        if (sizeof(Impact) != ImpactRecordSize
            || Marshal.OffsetOf<Impact>(nameof(Impact.Material)) != (nint)40
            || Marshal.OffsetOf<Impact>(nameof(Impact.OtherMaterial)) != (nint)44)
        {
            Refusal = $"impact record is {sizeof(Impact)} bytes, expected {ImpactRecordSize} with materials at 40 and 44";

            return false;
        }

        s_api = *api;
        s_ready = true;

        return true;
    }

    public static bool Ready => s_ready;

    private static byte* Utf8(string text, out IntPtr owned)
    {
        owned = Marshal.StringToCoTaskMemUTF8(text);

        return (byte*)owned;
    }

    public static uint DeclareComponent(
        void* world,
        string name,
        uint size,
        (string Name, FieldKind Kind, uint Offset)[] fields,
        string[] requirements
    )
    {
        byte* utf8 = Utf8(name, out IntPtr owned);

        var descriptors = new FieldDescriptor[fields.Length];
        var fieldNames = new IntPtr[fields.Length];
        var needed = new IntPtr[requirements.Length];

        try
        {
            for (int index = 0; index < fields.Length; ++index)
            {
                fieldNames[index] = Marshal.StringToCoTaskMemUTF8(fields[index].Name);

                descriptors[index] = new FieldDescriptor
                {
                    Name = (byte*)fieldNames[index],
                    Kind = (uint)fields[index].Kind,
                    Offset = fields[index].Offset
                };
            }

            for (int index = 0; index < requirements.Length; ++index)
            {
                needed[index] = Marshal.StringToCoTaskMemUTF8(requirements[index]);
            }

            fixed (FieldDescriptor* first = descriptors)
            fixed (IntPtr* required = needed)
            {
                return s_api.DeclareComponent(
                    world,
                    utf8,
                    size,
                    first,
                    (uint)descriptors.Length,
                    (byte**)required,
                    (uint)needed.Length
                );
            }
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);

            foreach (IntPtr held in fieldNames)
            {
                if (held != IntPtr.Zero)
                {
                    Marshal.FreeCoTaskMem(held);
                }
            }

            foreach (IntPtr held in needed)
            {
                if (held != IntPtr.Zero)
                {
                    Marshal.FreeCoTaskMem(held);
                }
            }
        }
    }

    public static uint FindComponent(void* world, string name)
    {
        byte* utf8 = Utf8(name, out IntPtr owned);

        try
        {
            return s_api.FindComponent(world, utf8);
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool SetComponent<T>(void* world, uint type, uint entity, in T value)
        where T : unmanaged
    {
        fixed (T* at = &value)
        {
            return s_api.SetComponent(world, type, entity, at) != 0;
        }
    }

    public static T* GetComponent<T>(void* world, uint type, uint entity)
        where T : unmanaged
    {
        return (T*)s_api.GetComponent(world, type, entity);
    }

    public static bool RemoveComponent(void* world, uint type, uint entity)
    {
        return s_api.RemoveComponent(world, type, entity) != 0;
    }

    public static uint ComponentCount(void* world, uint type)
    {
        return s_api.ComponentCount(world, type);
    }

    public static ReadOnlySpan<uint> Entities(void* world, uint type)
    {
        uint count = s_api.ComponentCount(world, type);
        uint* first = s_api.ComponentEntities(world, type);

        return count == 0 || first == null
            ? ReadOnlySpan<uint>.Empty
            : new ReadOnlySpan<uint>(first, (int)count);
    }

    public static Span<T> Components<T>(void* world, uint type) where T : unmanaged
    {
        uint count = s_api.ComponentCount(world, type);
        void* first = s_api.ComponentData(world, type);

        return count == 0 || first == null
            ? Span<T>.Empty
            : new Span<T>(first, (int)count);
    }

    public static bool Deform(
        void* world,
        uint entity,
        ref SdfEditDescriptor edit,
        out SdfDeformSummary result
    )
    {
        SdfDeformSummary filled = default;

        fixed (SdfEditDescriptor* at = &edit)
        {
            int ok = s_api.DeformSdf(world, entity, at, &filled);

            result = filled;

            return ok != 0;
        }
    }

    public static uint MaterialsUnder(
        void* world,
        uint entity,
        ref SdfEditDescriptor edit,
        Span<uint> materials
    )
    {
        fixed (SdfEditDescriptor* at = &edit)
        fixed (uint* first = materials)
        {
            return s_api.MaterialsUnderSdf(
                world, entity, at, first, (uint)materials.Length
            );
        }
    }

    public static float SampleSdf(void* world, uint entity, Vector3 point)
    {
        float* at = stackalloc float[3];

        at[0] = point.X;
        at[1] = point.Y;
        at[2] = point.Z;

        return s_api.SampleSdf(world, entity, at);
    }

    public static bool GetPose(void* world, uint entity, out Pose pose)
    {
        float* held = stackalloc float[7];

        int ok = s_api.GetPose(world, entity, held);

        pose = new Pose
        {
            Position = new Vector3(held[0], held[1], held[2]),
            Rotation = new Quaternion
            {
                X = held[3], Y = held[4], Z = held[5], W = held[6]
            }
        };

        return ok != 0;
    }

    public static bool SetPose(void* world, uint entity, in Pose pose)
    {
        float* held = stackalloc float[7];

        held[0] = pose.Position.X;
        held[1] = pose.Position.Y;
        held[2] = pose.Position.Z;
        held[3] = pose.Rotation.X;
        held[4] = pose.Rotation.Y;
        held[5] = pose.Rotation.Z;
        held[6] = pose.Rotation.W;

        return s_api.SetPose(world, entity, held) != 0;
    }

    public static uint Entities(void* world, Span<uint> buffer)
    {
        fixed (uint* first = buffer)
        {
            return s_api.Entities(world, first, (uint)buffer.Length);
        }
    }

    public static bool HasComponent(void* world, uint entity, string name)
    {
        byte* utf8 = Utf8(name, out IntPtr owned);

        try
        {
            return s_api.HasComponent(world, entity, utf8) != 0;
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool Bounds(
        void* world,
        uint entity,
        out Vector3 minimum,
        out Vector3 maximum
    )
    {
        float* low = stackalloc float[3];
        float* high = stackalloc float[3];

        int ok = s_api.Bounds(world, entity, low, high);

        minimum = new Vector3(low[0], low[1], low[2]);
        maximum = new Vector3(high[0], high[1], high[2]);

        return ok != 0;
    }

    public static void Log(string message)
    {
        byte* utf8 = Utf8(message, out IntPtr owned);

        try
        {
            s_api.Log(utf8);
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool KeyHeld(void* input, uint key)
        => s_api.KeyHeld(input, key) != 0;

    public static bool KeyPressed(void* input, uint key)
        => s_api.KeyPressed(input, key) != 0;

    public static bool KeyReleased(void* input, uint key)
        => s_api.KeyReleased(input, key) != 0;

    public static bool MouseHeld(void* input, uint button)
        => s_api.MouseHeld(input, button) != 0;

    public static bool MousePressed(void* input, uint button)
        => s_api.MousePressed(input, button) != 0;

    public static Vector2 Cursor(void* input)
    {
        float* xy = stackalloc float[2];

        s_api.Cursor(input, xy);

        return new Vector2(xy[0], xy[1]);
    }

    public static uint CreateSdfGrid(void* world, Pose pose, uint resolution, float margin)
    {
        float* seat = stackalloc float[7];

        seat[0] = pose.Position.X;
        seat[1] = pose.Position.Y;
        seat[2] = pose.Position.Z;
        seat[3] = pose.Rotation.X;
        seat[4] = pose.Rotation.Y;
        seat[5] = pose.Rotation.Z;
        seat[6] = pose.Rotation.W;

        uint* cells = stackalloc uint[3];

        cells[0] = resolution;
        cells[1] = resolution;
        cells[2] = resolution;

        return s_api.CreateSdfGrid(world, seat, cells, margin);
    }

    public static bool Destroy(void* world, uint entity)
        => s_api.Destroy(world, entity) != 0;

    public static bool Restart(void* world)
        => s_api.Restart(world) != 0;

    public static bool TryGetMaterial(void* world, uint index, out PhysicalMaterial material)
    {
        PhysicalMaterial found = default;

        int ok = s_api.Material(world, index, &found);

        material = found;

        return ok != 0;
    }

    public static bool AddPrimitive(void* world, uint entity, ref SdfEditDescriptor shape)
    {
        fixed (SdfEditDescriptor* described = &shape)
        {
            return s_api.AddPrimitive(world, entity, described) != 0;
        }
    }

    public static bool ShareSdf(void* world, uint donor, uint taker)
        => s_api.ShareSdf(world, donor, taker) != 0;

    public static uint MainCamera(void* world) => s_api.MainCamera(world);

    public static bool SetRigidBody(void* world, uint entity, float mass)
        => s_api.SetRigidBody(world, entity, mass) != 0;

    public static bool ScreenPointToRay(
        void* world,
        uint camera,
        Vector2 screen,
        Vector2 viewport,
        out Ray ray
    )
    {
        float* point = stackalloc float[2];
        float* size = stackalloc float[2];
        float* found = stackalloc float[6];

        point[0] = screen.X;
        point[1] = screen.Y;
        size[0] = viewport.X;
        size[1] = viewport.Y;

        if (s_api.ScreenPointToRay(world, camera, point, size, found) == 0)
        {
            ray = default;

            return false;
        }

        ray = new Ray(
            new Vector3(found[0], found[1], found[2]),
            new Vector3(found[3], found[4], found[5])
        );

        return true;
    }

    public static Vector2 Viewport(void* input)
    {
        float* wh = stackalloc float[2];

        s_api.Viewport(input, wh);

        return new Vector2(wh[0], wh[1]);
    }

    public static bool Raycast(void* queries, Ray ray, float maxDistance, out RaycastHit hit)
    {
        float* coordinates = stackalloc float[6] {
            ray.Origin.X, ray.Origin.Y, ray.Origin.Z,
            ray.Direction.X, ray.Direction.Y, ray.Direction.Z
        };
        RaycastHit found = default;
        int result = s_api.Raycast(queries, coordinates, maxDistance, &found);
        hit = found;
        if (result < 0)
        {
            throw new ArgumentException("Raycast requires a world handed to a running system, a finite ray, a nonzero direction and a nonnegative distance.");
        }
        return result == 1;
    }

    public static bool SetVelocity(void* world, uint entity, Vector3 velocity)
    {
        float* value = stackalloc float[3];

        value[0] = velocity.X;
        value[1] = velocity.Y;
        value[2] = velocity.Z;

        return s_api.SetVelocity(world, entity, value) != 0;
    }

    public static bool AddSystem(
        void* schedule,
        Phase phase,
        int order,
        string name,
        delegate* unmanaged<void*, SystemContext*, void> callback,
        void* user
    )
    {
        byte* utf8 = Utf8(name, out IntPtr owned);

        try
        {
            return s_api.AddSystem(
                schedule, (uint)phase, order, utf8, callback, user) != 0;
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool RemoveSystem(void* schedule, string name)
    {
        byte* utf8 = Utf8(name, out IntPtr owned);

        try
        {
            return s_api.RemoveSystem(schedule, utf8) != 0;
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool DrawLine(
        void* debug,
        Vector3 from,
        Vector3 to,
        Color colour,
        float seconds,
        string category,
        bool depthTested
    )
    {
        float* a = stackalloc float[3];
        float* b = stackalloc float[3];
        float* rgba = stackalloc float[4];

        a[0] = from.X;
        a[1] = from.Y;
        a[2] = from.Z;

        b[0] = to.X;
        b[1] = to.Y;
        b[2] = to.Z;

        rgba[0] = colour.R;
        rgba[1] = colour.G;
        rgba[2] = colour.B;
        rgba[3] = colour.A;

        byte* utf8 = Utf8(category, out IntPtr owned);

        try
        {
            return s_api.DrawLine(
                debug, a, b, rgba, seconds, utf8, depthTested ? 1 : 0) != 0;
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }

    public static bool ShowDebug(void* debug, string category, bool shown)
    {
        byte* utf8 = Utf8(category, out IntPtr owned);

        try
        {
            return s_api.ShowDebug(debug, utf8, shown ? 1 : 0) != 0;
        }
        finally
        {
            Marshal.FreeCoTaskMem(owned);
        }
    }
}
