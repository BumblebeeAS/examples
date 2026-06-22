# examples

Example applications for Bumblebee simulators, organized one subdirectory per
project. Each subdir is a self-contained sub-workspace (its own `packages/`,
`build.bash` / `run.bash`, `docker/`, and `*.repos` manifest):

| Directory | Simulator base | Contents |
|-----------|----------------|----------|
| [`bluerov/`](bluerov/) | `ardusub_sim:humble` | BlueROV2 / ArduSub missions, behaviour trees, perception (`bluerov_tasks`) |
| [`multivehicle/`](multivehicle/) | `multivehicle_sim:humble` | Multivehicle missions, BlueBoat control, PX4 offboard demo (`multivehicle_examples`) |

Pick the project you want and follow its README.

## Building

`colcon build` discovers `package.xml` files at any depth, so the per-project
nesting (`<project>/packages/...`) is found automatically. Because both projects'
packages live in the same repo but each pulls only its **own** dependencies (via
its `*.repos`), build the project you set up rather than the whole tree, e.g.:

```bash
# multivehicle:
colcon build --packages-up-to multivehicle_examples
# bluerov:
colcon build --packages-up-to bluerov_tasks
```

`--packages-up-to` builds the chosen package plus its dependencies and skips the
other project (whose deps you have not imported). Alternatively, drop a
`COLCON_IGNORE` file in the project subdir you are not building.
