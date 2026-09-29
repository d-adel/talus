using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct Color
{
    public float R;
    public float G;
    public float B;
    public float A;

    public Color(float r, float g, float b, float a = 1.0f)
    {
        R = r;
        G = g;
        B = b;
        A = a;
    }

    public static Color White => new(1.0f, 1.0f, 1.0f);

    public static Color Black => new(0.0f, 0.0f, 0.0f);

    public static Color Red => new(1.0f, 0.2f, 0.2f);

    public static Color Green => new(0.2f, 1.0f, 0.2f);

    public static Color Blue => new(0.3f, 0.5f, 1.0f);

    public static Color Yellow => new(1.0f, 0.9f, 0.2f);

    public static Color Orange => new(1.0f, 0.6f, 0.1f);

    public static Color Cyan => new(0.2f, 0.9f, 1.0f);
}
