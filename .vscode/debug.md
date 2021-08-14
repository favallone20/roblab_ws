# Debug

In order to acquire very basic information about integrating VSCode and ROS, visit [this page](https://medium.com/@tahsincankose/a-decent-integration-of-vscode-to-ros-4c1d951c982a).
It is preparatory to understand the content of this section.
In particular, the reader should know what a debug configuration in VSCode is and what pre-launch tasks are.

The focus of this section is on debugging ROS nodes, including test nodes.
Like other C++ executables, they are characterized by a main function, that is the entry point of the binary executable.
Unlike common C++ executables, ROS nodes usually require a ROS master to be up and running, therefore they often need to be launched with `roslaunch`.
In fact, this python command takes care of launching a master before the actual node executable is launched.
In addition, `roslaunch` can execute further operations, as demanded in the relating launch file, such as loading specific configuration files or placing parameters on the parameter server.

VSCode allows to debug C++ executables through a customizable debugger (usually GDB) and provides two configurations that are commonly referred to as `launch` and `attach`.
The former tells VSCode to spawn the process to debug (only compiled executables can be run, which is not the case of `rosrun` and `roslaunch`), the latter tells VSCode to attach to a running process, which, therefore, must exist before the debugger is executed, which is not, in general, the case of our ROS nodes.

The solution lies in the pre-launch tasks, that are processes that can be executed by VSCode before the debugger is executed.
Through a pre-launch task, we may run our `rosrun` or `roslaunch` command, so that the master executes, then the debugger takes care of launching the actual executable (or library, in case of plugins, dynamically-loaded libraries).
Since VSCode will wait for the pre-launch task to complete before calling the debugger, and the master must not terminate before the node is executed, it is necessary to configure the following option in the pre-launch task:

```json 
"isBackground": true
```

This will allow VSCode to run the pre-launch task in the background and will not wait for its termination before launching the debugger.
Sometimes, the option above makes VSCode complain that the process cannot be tracked.
If this happens, a `problemMatcher` needs to be configured:

```json
"problemMatcher": [
    {
        "pattern": [
            {
                "regexp": ".",
                "file": 1,
                "location": 2,
                "message": 3
            }
        ],
        "background":
        {
            "activeOnStart": true,
            "beginsPattern": ".",
            "endsPattern": ".",
        }
    }
]
```

The parameters used in the `problemMatcher` are completely random, but VSCode wants them to be defined anyway.

A complete example of a task configuration in `tasks.json` (to debug a test executable) is reported here below:

```json
"tasks": [
    {
        "label": "test_dynamic_programming_solver_totp",
        "type": "shell",
        "command": "roslaunch moveit_dp_redundancy_resolution test_dynamic_programming_solver_planar_2r.test debugger_attached:=true",
        "isBackground": true,
        "problemMatcher": [
            {
                "pattern": [
                    {
                        "regexp": ".",
                        "file": 1,
                        "location": 2,
                        "message": 3
                    }
                ],
                "background":
                {
                    "activeOnStart": true,
                    "beginsPattern": ".",
                    "endsPattern": ".",
                }
            }
        ]
    }
]
```

It is worth noticing, in the snippet above, the presence of a launch parameter called `debugger_attached`.
This parameter tells `roslaunch` that the node should not be spawned, because it will by the debugger.
This way, `roslaunch` will start the ROS master, load the needed parameters on the parameter server and will skip the actual execution of the node.
In order for this to work, the launch file should have an `unless` keyword in the `node` or `test` tag, e.g., for node,

```xml
<node name="demo_totpr_node" pkg="moveit_dp_redundancy_resolution" type="demo_totpr_node" output="screen" unless="$(arg debugger_attached)"/>
```

The `configurations` in `launch.json` will look like this:

```json
"configurations": [
    {
        "name": "(gdb) Launch",
        "type": "cppdbg",
        "request": "launch",
        "program": "${workspaceFolder}/../devel/.private/moveit_dp_redundancy_resolution/lib/moveit_dp_redundancy_resolution/dynamic_programming_solver_time_optimal_planning-test",
        "args":[],
        "stopAtEntry": true,
        "cwd": "${workspaceFolder}",
        "environment": [],
        "internalConsoleOptions": "openOnSessionStart",
        "externalConsole": false,
        "preLaunchTask": "test_dynamic_programming_solver_totp",
        "MIMode": "gdb",
        "setupCommands": [
            {
                "description": "Enable pretty-printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            },
        ]
    },
]
```

Running the configuration here called `(gdb) Launch` would suffice to attach the debugger to the process.

If you want to debug a plugin library, the process is the same, except that the `program` attribute of the configuration must point to the plugin library, e.g., for move group,

```json
"program": "/opt/ros/melodic/lib/moveit_ros_move_group/move_group"
```