using System.Runtime.InteropServices;

namespace Talus.Engine;

[StructLayout(LayoutKind.Sequential)]
public struct Vector2
{
    public float X;
    public float Y;

    public Vector2(float x, float y)
    {
        X = x;
        Y = y;
    }

    public static Vector2 operator +(Vector2 a, Vector2 b) => new(a.X + b.X, a.Y + b.Y);

    public static Vector2 operator -(Vector2 a, Vector2 b) => new(a.X - b.X, a.Y - b.Y);

    public static Vector2 operator -(Vector2 a) => new(-a.X, -a.Y);

    public static Vector2 operator *(Vector2 a, float s) => new(a.X * s, a.Y * s);

    public static Vector2 operator *(float s, Vector2 a) => a * s;

    public static Vector2 operator /(Vector2 a, float s) => new(a.X / s, a.Y / s);
}

[StructLayout(LayoutKind.Sequential)]
public struct Vector3
{
    public float X;
    public float Y;
    public float Z;

    public Vector3(float x, float y, float z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public static Vector3 Zero => new(0.0f, 0.0f, 0.0f);

    public static Vector3 operator +(Vector3 a, Vector3 b)
        => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static Vector3 operator -(Vector3 a, Vector3 b)
        => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Vector3 operator -(Vector3 a) => new(-a.X, -a.Y, -a.Z);

    public static Vector3 operator *(Vector3 a, float s) => new(a.X * s, a.Y * s, a.Z * s);

    public static Vector3 operator *(float s, Vector3 a) => a * s;

    public static Vector3 operator /(Vector3 a, float s) => new(a.X / s, a.Y / s, a.Z / s);

    public float LengthSquared => X * X + Y * Y + Z * Z;

    public float Length => MathF.Sqrt(LengthSquared);

    public Vector3 Normalized
    {
        get
        {
            float length = Length;

            return length > 0.0f ? this / length : this;
        }
    }

    public static float Dot(Vector3 a, Vector3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    public static Vector3 Cross(Vector3 a, Vector3 b)
        => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
}

[StructLayout(LayoutKind.Sequential)]
public struct Vector4
{
    public float X;
    public float Y;
    public float Z;
    public float W;
}

[StructLayout(LayoutKind.Sequential)]
public struct Quaternion
{
    public float X;
    public float Y;
    public float Z;
    public float W;

    public static Quaternion Identity => new() { X = 0.0f, Y = 0.0f, Z = 0.0f, W = 1.0f };
}
