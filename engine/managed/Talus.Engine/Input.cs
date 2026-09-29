namespace Talus.Engine;

public enum Key : uint
{
    Unknown = 0,

    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,

    Digit0, Digit1, Digit2, Digit3, Digit4,
    Digit5, Digit6, Digit7, Digit8, Digit9,

    Space,
    Enter,
    Escape,
    Tab,
    Backspace,
    Delete,

    Shift,
    Control,
    Alt,

    Left,
    Right,
    Up,
    Down,

    F1, F2, F3, F4, F5, F6,
    F7, F8, F9, F10, F11, F12
}

public enum MouseButton : uint
{
    Left,
    Right,
    Middle
}

public readonly unsafe struct Input
{
    private readonly void* m_handle;

    internal Input(void* handle)
    {
        m_handle = handle;
    }

    public bool Held(Key key) => Native.KeyHeld(m_handle, (uint)key);

    public bool Pressed(Key key) => Native.KeyPressed(m_handle, (uint)key);

    public bool Released(Key key) => Native.KeyReleased(m_handle, (uint)key);

    public bool Held(MouseButton button)
        => Native.MouseHeld(m_handle, (uint)button);

    public bool Pressed(MouseButton button)
        => Native.MousePressed(m_handle, (uint)button);

    public Vector2 Cursor => Native.Cursor(m_handle);

    public Vector2 Viewport => Native.Viewport(m_handle);
}
