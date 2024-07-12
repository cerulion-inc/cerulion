using Godot;

public partial class TestUI : CanvasLayer
{
    // Don't forget to rebuild the project so the editor knows about the new signal.

    [Signal]
    public delegate void StartGameEventHandler();
}