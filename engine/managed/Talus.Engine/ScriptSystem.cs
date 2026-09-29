namespace Talus.Engine;

public abstract class ScriptSystem
{
    public virtual Phase Phase => Phase.PostPhysics;

    public virtual int Order => 0;

    public virtual string Name => GetType().Name;

    public virtual void Run(World world)
    {
    }

    public virtual void Run(World world, Input input) => Run(world);
}
