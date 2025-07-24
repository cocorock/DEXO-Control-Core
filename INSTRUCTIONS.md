# Exoskeleton Control Workspace Setup Instructions

This file provides the steps to build the ROS workspace and launch the control system nodes.

## 1. Build the Workspace

First, you need to build the workspace using `catkin_make`. Open a terminal, navigate to the root of your workspace (`C:\Users\quepe\Documents\GitHub\DEXO-Control-Core`), and run the following command:

```bash
catkin_make
```

This command will create the `build` and `devel` directories in your workspace.

## 2. Source the Setup File

After the build process is complete, you need to source the `setup.bash` file in the `devel` directory. This will add your workspace to the ROS environment, allowing ROS to find your packages and nodes.

```bash
source devel/setup.bash
```

**Note:** You will need to do this for every new terminal you open.

## 3. Launch the Nodes

Now you can launch all the nodes using the launch file provided:

```bash
roslaunch exoskeleton_control exoskeleton_control.launch
```

This will start all the nodes defined in the launch file.

## 4. Using the Dummy Crutches Node

The `dummy_crutches` node will open in a new terminal window. You can use this terminal to send stop and calibration triggers to the system.

- To send a **stop trigger**, type `s` and press Enter.
- To send a **calibration trigger**, type `c` and press Enter.

## 5. Network Configuration for Multi-Machine Setup

To run ROS across multiple computers on the same network (e.g., to run the main control on the exoskeleton computer and visualize data on a separate desktop), you need to configure the ROS network environment variables.

Let's call the computer running the main launch file the **Master** and the computer for visualization the **Remote**.

### On the Master Computer:

1.  **Find its IP address.** On Windows, you can use `ipconfig` in the command prompt. On Linux/macOS, use `ifconfig` or `ip a`. Let's assume the Master's IP is `192.168.1.100`.

2.  **Set the `ROS_MASTER_URI` and `ROS_IP` environment variables.** Before running your launch file, execute these commands in your terminal. This tells all ROS nodes started in this terminal where to find the ROS Master and what IP address they should advertise.

    ```bash
    # Replace 192.168.1.100 with the Master's actual IP address
    export ROS_MASTER_URI=http://192.168.1.100:11311
    export ROS_IP=192.168.1.100
    ```

### On the Remote Computer:

1.  **Find its IP address.** Use the same method as on the Master. Let's assume the Remote's IP is `192.168.1.101`.

2.  **Set the `ROS_MASTER_URI` and `ROS_IP` environment variables.** The `ROS_MASTER_URI` **must point to the Master computer**. The `ROS_IP` should be the IP of the Remote computer itself.

    ```bash
    # ROS_MASTER_URI points to the Master computer
    export ROS_MASTER_URI=http://192.168.1.100:11311
    # ROS_IP is the IP of this Remote computer
    export ROS_IP=192.168.1.101
    ```

### Important Notes:

*   **Network Connectivity:** Ensure both computers can `ping` each other. If not, check your firewall settings and network connection.
*   **`.bashrc`:** For convenience, you can add the `export` commands to your `~/.bashrc` file on both machines so they are set automatically in every new terminal. Remember to `source ~/.bashrc` after editing.
*   **Name Resolution:** Both computers must be able to resolve each other's hostnames. If not, you can edit the `/etc/hosts` file on both machines to add entries for each other, for example:
    ```
    192.168.1.100 master-pc
    192.168.1.101 remote-pc
    ```
*   **Verification:** After launching the nodes on the Master, you should be able to run commands like `rostopic list` on the Remote computer and see all the topics published by the exoskeleton nodes. You can then use tools like `rviz` or `rqt_plot` to visualize the data.