using System.Runtime.InteropServices;

namespace Talus.Engine;

public static class ComponentTypes
{
    private static readonly Dictionary<Type, uint> s_types = new();

    public static uint TypeOf<T>() where T : unmanaged
    {
        if (s_types.TryGetValue(typeof(T), out uint type))
        {
            return type;
        }

        throw new InvalidOperationException(
            $"{typeof(T).Name} is not a component. Mark it [Component] and make "
            + "sure its assembly is the project's script assembly."
        );
    }

    internal static unsafe bool Declare(void* world, Type component)
    {
        var attribute =
            (ComponentAttribute?)Attribute.GetCustomAttribute(
                component, typeof(ComponentAttribute)
            );

        string name = attribute?.Name ?? component.Name;

        uint size = (uint)Marshal.SizeOf(component);

        var requirements = new List<string>();

        foreach (Attribute held in Attribute.GetCustomAttributes(
            component, typeof(RequireComponentAttribute)))
        {
            requirements.Add(((RequireComponentAttribute)held).Component);
        }

        uint type = Native.DeclareComponent(
            world, name, size, Layout.Describe(component), requirements.ToArray()
        );

        if (type == Native.InvalidComponent)
        {
            return false;
        }

        s_types[component] = type;

        return true;
    }

    internal static void Forget() => s_types.Clear();
}
