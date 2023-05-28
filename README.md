# AXES: yet another toolbox for computer graphics

**Major Features**: 

1. ECS based design: decoupling everything to entity-components;
2. Design For Performance: 
   1. Bunches of High Performance Libraries, e.g. abseil/Vulkan/Eigen/Taskflow/...
   2. Light-weight internal design.
3. (WIP) Node System Support
4. (WIP) GPU Compute Support

### Why ECS?

The idea of ECS:

> For any `System`, they just focus on one **SMALL** set of `Component`.

For example, 
1. GUI System just focus on `Rendering`.
2. Physics System just focus on `Simulation`
3. InterOp System just takes simulation result and generate `RenderData`

By introducing ECS, the readability and performance is improved.

## Requirements

1. c++1z supported, `gcc`/`clang` compiler, `msvc` may supported;
2. CMake, with vcpkg integration
3. Vulkan SDK

To build

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=${VCPKG_TOOLCHAIN_FILE}
```

## Details 

### ECS

In [ACG](https://github.com/Adversarr/ACG), we use a class `Mesh` to describe the render data, and we need to *submit* the data to the `Renderer` for rendering. However, we do not know, when and where will we generate the data.

For example, we want to render some mass-spring based cloth. We may create the cloth `Entity` in our simulator. In the traditional way, we shall write code like:

```cpp
auto mesh = gui.AddMesh();
mesh.SetVertPositions(position)
    .SetVertNormal(normal)
    .SetVertColor(color)
    .SetIndices(faces);

gui.SetRenderMesh(mesh);
```

The problem occurs at the last line. We need to keep the gui object even in our simulator code. Whenever you need to update the mesh, you need to call `Gui::SetRenderMesh` to update the data on GPU. That is UGLY because the simulator should not need to know the existence of `Gui`. If we use ECS, the update operation can be done in a more elegant way. First, we create the `Entity` to represent the cloth.

```cpp
auto cloth = World{}.CreateEntity();
cloth.AttachOrGet<MeshRenderData>()
    .SetVertPositions(position)
    .SetVertNormal(normal)
    .SetVertColor(color)
    .SetIndices(faces)
    .SetFlush();
```

And in the GuiSystem (more accurate, `MeshPipeline`):

```cpp
for (auto [ent, rd]: ecs::ComponentManager<MeshRenderData>{}) {
  // Flush the render data from `rd` to GPU
}
```

This can help decrease the compile time, because we do not need to include the GUI headers. Moreover, it can help you get your code more clear, because we do not need to hold the `Gui` object any more!



