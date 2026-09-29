using System.Reflection;
using System.Runtime.Loader;
using System.Runtime.InteropServices;

namespace Talus.Engine;

public static unsafe class Boot
{
    private static readonly List<ScriptSystem> s_systems = new();

    private static void* s_schedule;

    public static int Start(nint api, nint schedule, nint world, string scripts)
    {
        if (!Native.Bind((ApiTable*)api))
        {
            Console.Error.WriteLine($"Talus script host refused the api: {Native.Refusal}");

            return 2;
        }

        s_schedule = (void*)schedule;

        Assembly? assembly = LoadScripts(scripts);

        if (assembly == null)
        {
            Native.Log("no script assembly, running with no systems");

            return 1;
        }

        if (!DeclareComponents(assembly, (void*)world))
        {
            return 0;
        }

        if (!RegisterSystems(assembly))
        {
            return 0;
        }

        Native.Log($"registered {s_systems.Count} script system(s)");

        return 1;
    }

    [UnmanagedCallersOnly]
    internal static int Initialize(
        ApiTable* api,
        void* schedule,
        void* world,
        byte* scripts
    )
    {
        string directory = Marshal.PtrToStringUTF8((nint)scripts) ?? string.Empty;

        return Start((nint)api, (nint)schedule, (nint)world, directory);
    }

    private sealed class ScriptContext : AssemblyLoadContext
    {
        public ScriptContext() : base("talus.scripts", isCollectible: true)
        {
        }

        protected override Assembly? Load(AssemblyName name)
            => name.Name == typeof(Boot).Assembly.GetName().Name
                ? typeof(Boot).Assembly
                : null;
    }

    private static ScriptContext? s_context;

    public static void Stop()
    {
        if (s_schedule != null)
        {
            foreach (ScriptSystem system in s_systems)
            {
                Native.RemoveSystem(s_schedule, system.Name);
            }
        }

        s_systems.Clear();
        ComponentTypes.Forget();

        s_context?.Unload();
        s_context = null;
    }

    private static Assembly? LoadScripts(string directory)
    {
        if (string.IsNullOrEmpty(directory) || !Directory.Exists(directory))
        {
            return null;
        }

        string path = Path.Combine(directory, "Scripts.dll");

        if (!File.Exists(path))
        {
            return null;
        }

        try
        {
            Stop();

            string full = Path.GetFullPath(path);

            foreach (Assembly loaded in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (!loaded.IsDynamic && string.Equals(
                        loaded.Location, full, StringComparison.OrdinalIgnoreCase))
                {
                    return loaded;
                }
            }

            s_context = new ScriptContext();

            using var image = new MemoryStream(File.ReadAllBytes(path));
            string symbols = Path.ChangeExtension(path, ".pdb");

            if (File.Exists(symbols))
            {
                using var pdb = new MemoryStream(File.ReadAllBytes(symbols));

                return s_context.LoadFromStream(image, pdb);
            }

            return s_context.LoadFromStream(image);
        }
        catch (Exception error)
        {
            Native.Log("the script assembly did not load: " + error.Message);

            return null;
        }
    }

    private static bool DeclareComponents(Assembly assembly, void* world)
    {
        ComponentTypes.Forget();

        foreach (Type type in assembly.GetTypes())
        {
            if (!type.IsValueType
                || Attribute.GetCustomAttribute(type, typeof(ComponentAttribute))
                    == null)
            {
                continue;
            }

            if (!ComponentTypes.Declare(world, type))
            {
                Native.Log("component refused: " + type.Name);

                return false;
            }
        }

        return true;
    }

    private static bool RegisterSystems(Assembly assembly)
    {
        s_systems.Clear();

        foreach (Type type in assembly.GetTypes())
        {
            if (type.IsAbstract || !typeof(ScriptSystem).IsAssignableFrom(type))
            {
                continue;
            }

            if (Activator.CreateInstance(type) is not ScriptSystem system)
            {
                Native.Log("system refused: " + type.Name);

                return false;
            }

            s_systems.Add(system);
        }

        s_systems.Sort(
            (first, second) => first.Phase != second.Phase
                ? first.Phase.CompareTo(second.Phase)
                : first.Order.CompareTo(second.Order)
        );

        for (int index = 0; index < s_systems.Count; ++index)
        {
            if (!Native.AddSystem(
                    s_schedule,
                    s_systems[index].Phase,
                    s_systems[index].Order,
                    s_systems[index].Name,
                    &Run,
                    (void*)(nint)index
                ))
            {
                Native.Log("could not register " + s_systems[index].Name);

                return false;
            }
        }

        return true;
    }

    [UnmanagedCallersOnly]
    private static void Run(void* user, SystemContext* context)
    {
        int index = (int)(nint)user;

        if (index < 0 || index >= s_systems.Count || context == null
            || context->Size < sizeof(SystemContext))
        {
            return;
        }

        Debug.Attach(context->Debug);

        s_systems[index].Run(
            new World(
                context->World,
                context->DeltaSeconds,
                context->Frame,
                context->Impacts,
                context->ImpactCount,
                context->Queries
            ),
            new Input(context->Input)
        );
    }
}
