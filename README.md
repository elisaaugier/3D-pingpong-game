# 3D Ping-Pong Simulation

A real-time 3D ping-pong game built with C++ and OpenGL. The project combines a
rigid-body ball simulation, a rendered table and net, keyboard-controlled or
automatic paddle movement, match scoring, and an interactive camera.

## Features

- 3D scene rendered with OpenGL 3.3 and GLSL shaders
- Rigid-body ball simulation with gravity, collisions, and spin
- Manual or automatic paddle control
- Point and set scoring; the match ends when a player wins three sets
- Orbit, pan, zoom, and ball-follow camera controls
- Wireframe and surface rendering modes

## Requirements

- CMake 3.10 or later
- A C++11-compatible compiler
- An OpenGL 3.3-compatible graphics driver

GLFW, GLM, and GLAD are included in `dep/`; no separate package installation
for these libraries is required.

## Build and run

Run these commands from the repository root:

```sh
cmake -S . -B build
cmake --build build --config Release
```

The build copies the executable to the repository root. Run it from that
directory so the relative paths to shaders and textures resolve correctly:

```sh
./tpProjet
```

On Windows, run `tpProjet.exe` from the repository root. If CMake or the
compiler cannot find the platform's OpenGL development libraries, install the
appropriate system packages or SDK and configure the project again.

## Controls

| Input | Action |
| --- | --- |
| `P` | Pause or resume the simulation |
| `R` | Reset the current game (disabled after the match ends) |
| Arrow keys | Move the paddle when manual control is enabled |
| `K` or `L` | Toggle manual paddle control |
| `M` or `;` | Toggle the camera that follows the ball |
| Left mouse button + drag | Rotate the camera |
| Right mouse button + drag | Pan the camera |
| Middle mouse button + drag | Zoom |
| `W` | Switch to wireframe rendering |
| `F` | Switch to surface rendering |
| `S` | Save a screenshot |
| `H` | Print the controls in the terminal |
| `Esc` | Quit |

Manual paddle control is enabled by default. When it is disabled, the simulation
positions the paddle automatically.

## Project layout

```text
src/       Application, physics, mesh, camera, and shader code
data/      Textures used by the scene
dep/       Vendored GLFW, GLM, and GLAD dependencies
```
