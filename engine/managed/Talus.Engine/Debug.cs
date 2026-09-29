namespace Talus.Engine;

public static unsafe class Debug
{
    private const int Segments = 24;

    private static void* s_handle;

    internal static void Attach(void* handle) => s_handle = handle;

    public static void Log(string message) => Native.Log(message);

    public static bool DrawLine(
        Vector3 from,
        Vector3 to,
        Color colour,
        float seconds = 0.0f,
        string category = "script",
        bool depthTested = true
    )
        => s_handle != null
           && Native.DrawLine(s_handle, from, to, colour, seconds, category, depthTested);

    public static bool DrawRay(
        Vector3 origin,
        Vector3 along,
        Color colour,
        float seconds = 0.0f,
        string category = "script",
        bool depthTested = true
    )
        => DrawLine(origin, origin + along, colour, seconds, category, depthTested);

    public static bool DrawPoint(
        Vector3 at,
        float size,
        Color colour,
        float seconds = 0.0f,
        string category = "script",
        bool depthTested = true
    )
    {
        float half = size * 0.5f;

        return DrawLine(at - new Vector3(half, 0.0f, 0.0f), at + new Vector3(half, 0.0f, 0.0f), colour, seconds, category, depthTested)
            & DrawLine(at - new Vector3(0.0f, half, 0.0f), at + new Vector3(0.0f, half, 0.0f), colour, seconds, category, depthTested)
            & DrawLine(at - new Vector3(0.0f, 0.0f, half), at + new Vector3(0.0f, 0.0f, half), colour, seconds, category, depthTested);
    }

    public static bool DrawBox(
        Vector3 minimum,
        Vector3 maximum,
        Color colour,
        float seconds = 0.0f,
        string category = "script",
        bool depthTested = true
    )
    {
        bool drawn = true;

        for (int bits = 0; bits != 8; ++bits)
        {
            for (int axis = 0; axis != 3; ++axis)
            {
                int flag = 1 << axis;

                if ((bits & flag) == 0)
                {
                    drawn &= DrawLine(
                        Corner(minimum, maximum, bits),
                        Corner(minimum, maximum, bits | flag),
                        colour,
                        seconds,
                        category,
                        depthTested
                    );
                }
            }
        }

        return drawn;
    }

    public static bool DrawSphere(
        Vector3 centre,
        float radius,
        Color colour,
        float seconds = 0.0f,
        string category = "script",
        bool depthTested = true
    )
    {
        bool drawn = true;

        for (int plane = 0; plane != 3; ++plane)
        {
            Vector3 previous = centre + Axis((plane + 1) % 3) * radius;

            for (int step = 1; step <= Segments; ++step)
            {
                float angle = MathF.Tau * step / Segments;

                Vector3 next = centre
                    + Axis((plane + 1) % 3) * (radius * MathF.Cos(angle))
                    + Axis((plane + 2) % 3) * (radius * MathF.Sin(angle));

                drawn &= DrawLine(previous, next, colour, seconds, category, depthTested);

                previous = next;
            }
        }

        return drawn;
    }

    public static bool Show(string category, bool shown)
        => s_handle != null && Native.ShowDebug(s_handle, category, shown);

    private static Vector3 Axis(int index)
        => index == 0 ? new Vector3(1.0f, 0.0f, 0.0f)
            : index == 1 ? new Vector3(0.0f, 1.0f, 0.0f)
            : new Vector3(0.0f, 0.0f, 1.0f);

    private static Vector3 Corner(Vector3 minimum, Vector3 maximum, int bits)
        => new(
            (bits & 1) != 0 ? maximum.X : minimum.X,
            (bits & 2) != 0 ? maximum.Y : minimum.Y,
            (bits & 4) != 0 ? maximum.Z : minimum.Z
        );
}
