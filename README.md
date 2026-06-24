# examples

Example applications for Bumblebee simulators.

## Sub-examples

- [BlueROV examples](bluerov/README.md)
- [Multivehicle examples](multivehicle/README.md)

## Demo videos

### BlueROV missions

- Bin mission: https://github.com/user-attachments/assets/6c262df8-bac6-492a-aef1-9e8cfc30d8a8
- Torpedo mission: https://github.com/user-attachments/assets/9a9c25c5-637a-403a-b34d-4048f9afb5e0

## Building

Build only the project you imported dependencies for:

```bash
# multivehicle
colcon build --packages-up-to multivehicle_examples

# bluerov
colcon build --packages-up-to bluerov_tasks
```
