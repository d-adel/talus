using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Talus.Engine;

public readonly unsafe struct Entity : IEquatable<Entity>
{
    internal Entity(uint id)
    {
        Id = id;
    }

    public uint Id { get; }

    public static Entity None => new(uint.MaxValue);

    public bool Valid => Id != uint.MaxValue;

    public bool Equals(Entity other) => Id == other.Id;

    public override bool Equals(object? other) => other is Entity e && Equals(e);

    public override int GetHashCode() => Id.GetHashCode();

    public override string ToString() => Id.ToString();
}

public readonly unsafe struct World
{
    private readonly void* m_handle;

    private readonly float m_delta;
    private readonly uint m_frame;

    private readonly Impact* m_impacts;
    private readonly uint m_impactCount;
    private readonly void* m_queries;

    internal World(
        void* handle,
        float delta = 0.0f,
        uint frame = 0u,
        Impact* impacts = null,
        uint impactCount = 0u,
        void* queries = null
    )
    {
        m_handle = handle;
        m_delta = delta;
        m_frame = frame;
        m_impacts = impacts;
        m_impactCount = impactCount;
        m_queries = queries;
    }

    internal void* Handle => m_handle;

    public float DeltaSeconds => m_delta;

    public uint Frame => m_frame;

    public ReadOnlySpan<Impact> Impacts
        => m_impacts == null || m_impactCount == 0u
            ? ReadOnlySpan<Impact>.Empty
            : new ReadOnlySpan<Impact>(m_impacts, (int)m_impactCount);

    public ReadOnlySpan<Entity> Entities<T>() where T : unmanaged
    {
        ReadOnlySpan<uint> ids = Native.Entities(m_handle, ComponentTypes.TypeOf<T>());

        return MemoryMarshal.Cast<uint, Entity>(ids);
    }

    public Span<T> Components<T>() where T : unmanaged
    {
        return Native.Components<T>(m_handle, ComponentTypes.TypeOf<T>());
    }

    public int Count<T>() where T : unmanaged
    {
        return (int)Native.ComponentCount(m_handle, ComponentTypes.TypeOf<T>());
    }

    public bool Has<T>(Entity entity) where T : unmanaged
    {
        return Native.GetComponent<T>(
            m_handle, ComponentTypes.TypeOf<T>(), entity.Id
        ) != null;
    }

    public ref T Get<T>(Entity entity) where T : unmanaged
    {
        T* at = Native.GetComponent<T>(
            m_handle, ComponentTypes.TypeOf<T>(), entity.Id
        );

        if (at == null)
        {
            throw new InvalidOperationException(
                $"Entity {entity} has no {typeof(T).Name}"
            );
        }

        return ref Unsafe.AsRef<T>(at);
    }

    public bool TryGet<T>(Entity entity, out T value) where T : unmanaged
    {
        T* at = Native.GetComponent<T>(
            m_handle, ComponentTypes.TypeOf<T>(), entity.Id
        );

        value = at == null ? default : *at;

        return at != null;
    }

    public void Set<T>(Entity entity, in T value) where T : unmanaged
    {
        Native.SetComponent(
            m_handle, ComponentTypes.TypeOf<T>(), entity.Id, value
        );
    }

    public bool Remove<T>(Entity entity) where T : unmanaged
    {
        return Native.RemoveComponent(
            m_handle, ComponentTypes.TypeOf<T>(), entity.Id
        );
    }

    public Query<T> Query<T>() where T : unmanaged => new(this);

    public bool Deform(Entity entity, in SdfEdit edit, out DeformResult result)
    {
        SdfEditDescriptor descriptor = edit.ToDescriptor();

        bool ok = Native.Deform(
            m_handle, entity.Id, ref descriptor, out SdfDeformSummary summary
        );

        result = new DeformResult(summary.CellsRemoved, summary.VolumeRemoved);

        return ok;
    }

    public int MaterialsUnder(Entity entity, in SdfEdit edit, Span<uint> materials)
    {
        SdfEditDescriptor descriptor = edit.ToDescriptor();

        return (int)Native.MaterialsUnder(
            m_handle, entity.Id, ref descriptor, materials
        );
    }

    public Entity CreateSdfGrid(Pose pose, uint resolution, float margin = 0.0f)
        => new Entity(Native.CreateSdfGrid(m_handle, pose, resolution, margin));

    public bool Destroy(Entity entity)
        => Native.Destroy(m_handle, entity.Id);

    public bool Restart()
        => Native.Restart(m_handle);

    public bool TryGetMaterial(uint index, out PhysicalMaterial material)
        => Native.TryGetMaterial(m_handle, index, out material);

    public bool AddPrimitive(Entity entity, in SdfEdit shape)
    {
        SdfEditDescriptor descriptor = shape.ToDescriptor();

        return Native.AddPrimitive(m_handle, entity.Id, ref descriptor);
    }

    public bool ShareSdf(Entity donor, Entity taker)
        => Native.ShareSdf(m_handle, donor.Id, taker.Id);

    public Entity MainCamera
    {
        get
        {
            uint id = Native.MainCamera(m_handle);

            return id == uint.MaxValue ? Entity.None : new Entity(id);
        }
    }

    public bool SetRigidBody(Entity entity, float mass)
        => Native.SetRigidBody(m_handle, entity.Id, mass);

    public bool ScreenPointToRay(
        Entity camera,
        Vector2 screen,
        Vector2 viewport,
        out Ray ray
    )
        => Native.ScreenPointToRay(
            m_handle, camera.Id, screen, viewport, out ray
        );

    public bool SetVelocity(Entity entity, Vector3 velocity)
        => Native.SetVelocity(m_handle, entity.Id, velocity);

    public bool Has(Entity entity, string component)
        => Native.HasComponent(m_handle, entity.Id, component);

    public bool TryGetBounds(Entity entity, out Vector3 minimum, out Vector3 maximum)
        => Native.Bounds(m_handle, entity.Id, out minimum, out maximum);

    public bool Raycast(Ray ray, out RaycastHit hit, float maxDistance = float.PositiveInfinity)
        => Native.Raycast(m_queries, ray, maxDistance, out hit);

    public float SampleSdf(Entity entity, Vector3 point)
        => Native.SampleSdf(m_handle, entity.Id, point);

    public bool TryGetPose(Entity entity, out Pose pose)
        => Native.GetPose(m_handle, entity.Id, out pose);

    public bool SetPose(Entity entity, in Pose pose)
        => Native.SetPose(m_handle, entity.Id, pose);

    public Entity[] All()
    {
        uint total = Native.Entities(m_handle, Span<uint>.Empty);

        if (total == 0u)
        {
            return Array.Empty<Entity>();
        }

        uint[] ids = new uint[total];

        Native.Entities(m_handle, ids);

        Entity[] found = new Entity[total];

        for (int index = 0; index < found.Length; ++index)
        {
            found[index] = new Entity(ids[index]);
        }

        return found;
    }

    public static void Log(string message) => Native.Log(message);
}
