using Godot;
using System;

public partial class FileImportWindow : Control
{
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		ConnectFilesImportedSignal();
	}

	private void ConnectFilesImportedSignal() {
		var root = GetTree().Root;
		root.FilesDropped += OnFilesImported;
	}
	
	public void OnFilesImported(String[] files) {
		GD.Print("The following files were imported: " + files[0]);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
