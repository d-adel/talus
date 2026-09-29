using Talus.Engine;

namespace Demo;

public sealed class RestartSystem : ScriptSystem
{
    public override string Name => "restart";

    public override void Run(World world, Input input)
    {
        if (input.Pressed(Key.R))
        {
            world.Restart();
        }
    }
}
