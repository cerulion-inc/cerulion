# Godot Options Menus
For Godot 4.2

This plugin has options menus that aim to be easy to customize and persist settings in a user's config file.

[Example on itch.io](https://maaack.itch.io/godot-game-template)  
_Example is of [Maaack's Game Template](https://github.com/Maaack/Godot-Game-Template), which includes additional features._


#### Videos

[![Quick Intro Video](https://img.youtube.com/vi/U9CB3vKINVw/hqdefault.jpg)](https://youtu.be/U9CB3vKINVw)  
[![Installation Video](https://img.youtube.com/vi/-QWJnZ8bVdk/hqdefault.jpg)](https://youtu.be/-QWJnZ8bVdk)  
[All videos](/addons/maaacks_options_menus/docs/Videos.md)

#### Screenshots

![Key Rebinding](/addons/maaacks_options_menus/media/Screenshot-3-2.png)  
![Key Rebinding Confirmation](/addons/maaacks_options_menus/media/Screenshot-4-2.png) 
![Audio Controls](/addons/maaacks_options_menus/media/Screenshot-3-4.png)  
![Video Controls](/addons/maaacks_options_menus/media/Screenshot-4-3.png) 
[All screenshots](/addons/maaacks_options_menus/docs/Screenshots.md)

## Use Case
Setup options menus and accessibility features in about 15 minutes.

The core components can support a larger project, but the template was originally built to support smaller projects and game jams.

## Features

### Base

The `base/` folder holds the core components of the menus application.

-   Options Menus
-   Persistent Settings
-   Simple Config Interface
-   Keyboard/Mouse Support
-   Gamepad Support


### How it Works
- `AppConfig.tscn` is set as the first autoload. It calls `AppSettings.gd` to load all the configuration settings from the config file (if it exists) through `Config.gd`.
- `OptionControl.tscn` and its inherited scenes are used for most configurable options in the menus. They work with `Config.gd` to keep settings persistent between runs.
  
## Installation

### Godot Asset Library
This package is available as a plugin, meaning it can be added to an existing project. 

![Package Icon](/addons/maaacks_options_menus/media/Options-Icon-black-transparent-256x256.png)  

When editing an existing project:

1.  Go to the `AssetLib` tab.
2.  Search for "Maaack's Options Menus".
3.  Click on the result to open the plugin details.
4.  Click to Download.
5.  Check that contents are getting installed to `addons/` and there are no conflicts.
6.  Click to Install.
7.  Reload the project (you may see errors before you do this).
8.  Enable the plugin from the Project Settings > Plugins tab.  
    If it's enabled for the first time,
    1.  A dialogue window will appear asking to copy the example scenes out of `addons/`.
9.  Continue with the [Existing Project Instructions](/addons/maaacks_options_menus/docs/ExistingProject.md)  


### GitHub


1.  Download the latest release version from [GitHub](https://github.com/Maaack/Godot-Menus-Template/releases/latest).  
2.  Extract the contents of the archive.
3.  Move the `addons/maaacks_options_menus` folder into your project's `addons/` folder.  
4.  Open/Reload the project.  
5.  Enable the plugin from the Project Settings > Plugins tab.  
    If it's enabled for the first time,
    1.  A dialogue window will appear asking to copy the example scenes out of `addons/`.
6.  Continue with the [Existing Project Instructions](/addons/maaacks_options_menus/docs/ExistingProject.md) 

#### Extras

Users that want additional features can try [Maaack's Game Template](https://github.com/Maaack/Godot-Game-Template).  

## Usage

Changes can be made directly to scenes and scripts outside of `addons/`. 

A copy of the `examples/` directory is made outside of `addons/` when the plugin is enabled for the first time. However, if this is skipped, it is recommended developers inherit from scenes they want to use, and save the inherited scene outside of `addons/`. This avoids changes getting lost either from the package updating, or because of a `.gitignore`.

### Existing Project

[Existing Project Instructions](/addons/maaacks_options_menus/docs/ExistingProject.md)  


## Links
[Attribution](/addons/maaacks_options_menus/ATTRIBUTION.md)  
[License](/addons/maaacks_options_menus/LICENSE.txt)  
[Godot Asset Library - Plugin](https://godotengine.org/asset-library/asset/3058) 