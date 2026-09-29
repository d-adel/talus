using Talus.Engine;

namespace Demo;

public sealed class ShootingSystem : ScriptSystem
{
    private const float Speed = 22.0f;
    private const float Mass = 150.0f;
    private const float Radius = 0.35f;
    private const float Muzzle = 1.6f;
    private const float Height = 2.5f;
    private const uint Resolution = 33u;
    private const uint Material = 1u;

    public override int Order => 0;

    public override string Name => "shooting";

    public override void Run(World world, Input input)
    {
        bool aimed = input.Pressed(MouseButton.Left);

        if (!aimed && !input.Pressed(Key.F))
        {
            return;
        }

        Entity eye = world.MainCamera;

        if (!eye.Valid)
        {
            return;
        }

        if (!world.TryGetPose(eye, out Pose seat))
        {
            return;
        }

        Vector3 aim = Forward(seat.Rotation);

        if (aimed
            && world.ScreenPointToRay(eye, input.Cursor, input.Viewport, out Ray ray))
        {
            aim = ray.Direction;
        }

        Vector3 from = new(
            seat.Position.X + aim.X * Muzzle,
            Height + aim.Y * Muzzle,
            seat.Position.Z + aim.Z * Muzzle
        );

        Fire(world, from, new Vector3(aim.X * Speed, aim.Y * Speed, aim.Z * Speed));
    }

    private static void Fire(World world, Vector3 from, Vector3 velocity)
    {
        Pose seat = new() { Position = from, Rotation = Quaternion.Identity };

        Entity ball = world.CreateSdfGrid(seat, Resolution);

        if (!ball.Valid)
        {
            Debug.Log("shot refused: the world would not make a grid");

            return;
        }

        if (!world.AddPrimitive(ball, SdfEdit.Sphere(from, Radius).Adding(Material)))
        {
            world.Destroy(ball);

            Debug.Log("shot refused: the primitive pool is full");

            return;
        }

        world.SetRigidBody(ball, Mass);
        world.SetVelocity(ball, velocity);

        Debug.Log(
            $"shot {ball.Id} from {from.X:0.0} {from.Y:0.0} {from.Z:0.0}"
            + $" toward {velocity.X:0.0} {velocity.Y:0.0} {velocity.Z:0.0}"
        );
    }

    private static Vector3 Forward(Quaternion turn)
    {
        float x = turn.X;
        float y = turn.Y;
        float z = turn.Z;
        float w = turn.W;

        return new Vector3(
            -2.0f * (x * z + w * y),
            -2.0f * (y * z - w * x),
            -(1.0f - 2.0f * (x * x + y * y))
        );
    }
}
