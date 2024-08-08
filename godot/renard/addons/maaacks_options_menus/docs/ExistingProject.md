# Existing Project


1.  Add readable names for input actions to the controls menu.
    

    1.  Open `InputOptionsMenu.tscn` (or `MasterOptionsMenu`, which contains an instance of the scene).
    2.  Select the `Controls` node.
    3.  Update the `Action Name Map` to show readable names for the project's input actions.  
        1.  The keys are the project's input action names, while the values are the names shown in the controls menu.  
        2.  An example is provided. It can be updated or removed, either in the inspector for the node, or in the code of `InputOptionsMenu.gd`.  
    4.  Save the scene.  


2.  Add / remove configurable settings to / from menus.
    

    1.  Open `MiniOptionsMenu.tscn` or `[Audio|Visual|Input|Game]OptionsMenu.tscn` scenes to edit their options.
    2.  If an option is not desired, it can always be hidden, or removed entirely (sometimes with some additional work).
    3.  If a new option is desired, it can be added without writing code.
        1.  Find the node that contains the existing list of options. Usually, it's a `VBoxContainer`.
        2.  Add an `OptionControl.tscn` node as a child to the container.
            1.  `SliderOptionControl.tscn` or `ToggleOptionControl.tscn` can be used if those types match requirements. In that case, skip step 6.
            2.  `ListOptionControl.tscn` and `Vector2ListOptionControl.tscn` are also available, but more complicated. See the `ScreenResolution` example.
        3.  Select the `OptionControl` node just added, to edit it in the inspector.
        4.  Add an `Option Name`. This prefills the `Key` string.
        5.  Select an `Option Section`. This prefills the `Section` string.
        6.  Add any kind of `Button`, `Slider`, `LineEdit`, or `TextEdit` to the `OptionControl` node.
        7.  Save the scene.
    4.  For options to have an effect outside of the menu, it will need to be referenced by its `key` and `section` from `Config.gd`.
        1.  `Config.get_config(section, key, default_value)`
    5.  Validate the values being stored in your local `config.cfg` file.
        1.  Refer to [Accessing Persistent User Data User](https://docs.godotengine.org/en/stable/tutorials/io/data_paths.html#accessing-persistent-user-data-user) to find Godot user data on your machine.
        2.  Find the directory that matches your project's name.  
        3.  `config.cfg` should be in the top directory of the project.
