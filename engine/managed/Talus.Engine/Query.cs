namespace Talus.Engine;

public readonly unsafe ref struct Query<T> where T : unmanaged
{
    private readonly World m_world;

    internal Query(World world)
    {
        m_world = world;
    }

    public Enumerator GetEnumerator() => new(m_world);

    public unsafe ref struct Enumerator
    {
        private readonly World m_world;
        private readonly Entity[] m_entities;
        private readonly uint m_type;

        private int m_at;

        internal Enumerator(World world)
        {
            m_world = world;
            m_entities = world.Entities<T>().ToArray();
            m_type = ComponentTypes.TypeOf<T>();
            m_at = -1;
        }

        public bool MoveNext()
        {
            while (++m_at < m_entities.Length)
            {
                if (Lookup() != null)
                {
                    return true;
                }
            }

            return false;
        }

        public Row Current => new(m_entities[m_at], ref *Lookup());

        private T* Lookup()
        {
            return Native.GetComponent<T>(
                m_world.Handle, m_type, m_entities[m_at].Id
            );
        }
    }

    public readonly ref struct Row
    {
        internal Row(Entity entity, ref T value)
        {
            Entity = entity;
            Value = ref value;
        }

        public Entity Entity { get; }

        public readonly ref T Value;

        public void Deconstruct(out Entity entity, out T value)
        {
            entity = Entity;
            value = Value;
        }
    }
}
