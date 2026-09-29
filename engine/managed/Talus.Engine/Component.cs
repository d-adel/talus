namespace Talus.Engine;

[AttributeUsage(AttributeTargets.Struct)]
public sealed class ComponentAttribute : Attribute
{
    public ComponentAttribute(string? name = null)
    {
        Name = name;
    }

    public string? Name { get; }
}

[AttributeUsage(AttributeTargets.Struct, AllowMultiple = true)]
public sealed class RequireComponentAttribute : Attribute
{
    public RequireComponentAttribute(string component)
    {
        Component = component;
    }

    public string Component { get; }
}
