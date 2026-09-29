using System.Reflection;
using System.Runtime.InteropServices;

namespace Talus.Engine;

internal static class Layout
{
    internal static FieldKind KindOf(Type type)
    {
        if (type == typeof(float)) return FieldKind.Float;
        if (type == typeof(int)) return FieldKind.Int;
        if (type == typeof(uint)) return FieldKind.UInt;
        if (type == typeof(bool)) return FieldKind.Bool;
        if (type == typeof(Vector2)) return FieldKind.Vec2;
        if (type == typeof(Vector3)) return FieldKind.Vec3;
        if (type == typeof(Vector4)) return FieldKind.Vec4;
        if (type == typeof(Quaternion)) return FieldKind.Quat;

        throw new NotSupportedException(
            $"A component field of type {type.Name} cannot cross to the engine. "
            + "Use float, int, uint, bool, Vector2, Vector3, Vector4 or Quaternion."
        );
    }

    internal static (string Name, FieldKind Kind, uint Offset)[] Describe(Type component)
    {
        FieldInfo[] fields = component.GetFields(
            BindingFlags.Public | BindingFlags.Instance
        );

        var described = new (string, FieldKind, uint)[fields.Length];

        for (int index = 0; index < fields.Length; ++index)
        {
            FieldInfo field = fields[index];

            described[index] = (
                Camel(field.Name),
                KindOf(field.FieldType),
                (uint)Marshal.OffsetOf(component, field.Name)
            );
        }

        return described;
    }

    private static string Camel(string name)
    {
        return name.Length == 0
            ? name
            : char.ToLowerInvariant(name[0]) + name.Substring(1);
    }
}
