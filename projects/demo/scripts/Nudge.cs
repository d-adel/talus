using Talus.Engine;

namespace Demo;

[RequireComponent("RigidBody")]
[Component]
public struct Nudge
{
    public Vector3 Offset;
}

public sealed class NudgeSystem : ScriptSystem
{
    public override int Order => 100;

    public override string Name => "nudge";

    public override void Run(World world)
    {
        if (world.Count<Nudge>() == 0)
        {
            return;
        }

        foreach (Entity entity in world.Entities<Nudge>().ToArray())
        {
            if (world.TryGet(entity, out Nudge nudge) && world.TryGetPose(entity, out Pose pose))
            {
                pose.Position += nudge.Offset;
                world.SetPose(entity, pose);
            }

            world.Remove<Nudge>(entity);
        }
    }
}
