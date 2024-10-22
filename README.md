<a id="readme-top"></a>
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/cerulion-inc/cerulion">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="media/logo/Logo White Icon.svg">
    <img src="media/logo/Logo Main Icon.svg" alt="Logo" width="100" height="100">
    </picture>
  </a>

  <h3 align="center">Cerulion</h3>

  <p align="center">
    An interface for your robots that can [visualize] [plot] [debug] [control].
    <br/>
    Cross-platform. Low-latency. ROS-compatible.
    <br />
    <a href="https://github.com/cerulion-inc/cerulion/wiki"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/cerulion-inc/cerulion">View Demo</a>
    ·
    <a href="https://github.com/cerulion-inc/cerulion/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/cerulion-inc/cerulion/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![cerulion_screenshot][cerulion_screenshot]](https://cerulion.com)

Cerulion is your all-in-one, cross-platform solution for visualizing, debugging, plotting, logging, and controlling your robot. Whether you're working on a Mac, Windows PC, or a Linux workstation, Cerulion frees you from platform constraints, enabling seamless robot interfacing across all major systems.
  
Designed with flexibility and ease of use in mind, Cerulion provides a unified interface for developers to start debugging and visualizing their robots right away. With built-in low-latency communication and full compatibility with ROS, Cerulion ensures that your robot operations are smooth, efficient, and ready for the demands of modern robotics.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

Drivers will need to be installed on your robot platform, but for the interface frontend, binaries can be run immediately for MacOS, Windows, and Linux operating systems.

### Installation

##### On your computer

Download the application executable from [Releases](https://github.com/cerulion-inc/cerulion/releases) for your operating system.

##### On your robot

Currently, we use [LCM (Lightweight Communications and Marshaling)]([url](http://lcm-proj.github.io/lcm/)) for our backend. Very soon, we will also offer [IceOryx]([url](https://iceoryx.io)) (intra-machine IPC) + [Zenoh]([url](https://zenoh.io)) (inter-machine network communications) for bleeding edge improvements in latency using Rust.

You can start using LCM with Python on your robot right away after a simple `pip install lcm`. However, if you want to use ROS2 with LCM, we are developing a bridge to make this easier. Your best option for now is to use [this]([url](https://github.com/nrjl/lcm_to_ros)) bidirectional message subscriber/republisher.

If you just want to test without deploying on a robot, see [Usage](#Usage)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage
#### Loading your robot
https://github.com/user-attachments/assets/3a858ad9-ecaa-4940-8ea5-9eb177a47a7c

<!-- #### Checking communication status

#### Plotting sensor data

#### Publishing control commands -->


_For more examples, please refer to the [Documentation](https://github.com/cerulion-inc/cerulion/wiki)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With
[Godot Engine](https://godotengine.org)

[IceOryx2](https://iceoryx.io)

[Zenoh](https://zenoh.io)

[LCM](http://lcm-proj.github.io/lcm/#)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [x] Release v0.1
- [ ] Simplify LCM-ROS2 integration
- [ ] Add ZenOryx backend (Zenoh + IceOryx/IceOryx2)
- [ ] Dynamic content panels
- [ ] Basic simulation capabilities
- [ ] Basic controllers


See the [open issues](https://github.com/cerulion-inc/cerulion/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Top contributors:

<a href="https://github.com/cerulion-inc/cerulion/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cerulion-inc/cerulion" alt="" />
</a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GPLv3 License. See `LICENSE.txt` for more information.

Cerulion is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Cerulion is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Cerulion. If not, see <https://www.gnu.org/licenses/>.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Lakshay Sharma - lakshay@cerulion.com

Se Hwan Jeon - sehwan@cerulion.com

Project Link: [https://github.com/cerulion-inc/cerulion](https://github.com/cerulion-inc/cerulion)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

<!-- GITHUB METRICS -->
[contributors-shield]: https://img.shields.io/github/contributors/cerulion-inc/cerulion.svg?style=for-the-badge
[contributors-url]: https://github.com/cerulion-inc/cerulion/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/cerulion-inc/cerulion.svg?style=for-the-badge
[forks-url]: https://github.com/cerulion-inc/cerulion/network/members
[stars-shield]: https://img.shields.io/github/stars/cerulion-inc/cerulion.svg?style=for-the-badge
[stars-url]: https://github.com/cerulion-inc/cerulion/stargazers
[issues-shield]: https://img.shields.io/github/issues/cerulion-inc/cerulion.svg?style=for-the-badge
[issues-url]: https://github.com/cerulion-inc/cerulion/issues
[license-shield]: https://img.shields.io/github/license/cerulion-inc/cerulion?style=for-the-badge
[license-url]: https://github.com/cerulion-inc/cerulion/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/company/cerulion
[cerulion_screenshot]: media/images/cerulion_screenshot.png

<!-- BADGES -->
[Godot]: https://img.shields.io/badge/Godot-v4.3-%23478cbf?logo=godot-engine&logoColor=white
[Godot-badge-url]: https://godotengine.org
[IceOryx]: https://avatars.githubusercontent.com/u/69006087?s=48&v=4
[IceOryx-badge-url]: https://iceoryx.io/v2.0.2/
[Zenoh]: https://img.shields.io/badge/Godot-v4.3-%23478cbf?logo=godot-engine&logoColor=white
[Zenoh-badge-url]: https://godotengine.org
[LCM]: https://img.shields.io/badge/Godot-v4.3-%23478cbf?logo=godot-engine&logoColor=white
[LCM-badge-url]: https://godotengine.org
