# iRobot Create Simulator Toolbox

A comprehensive MATLAB-based simulator for the iRobot Create® platform. This toolbox allows you to simulate, visualize, and test autonomous control programs in a customizable virtual environment.

![Simulator Screenshot](https://raw.githubusercontent.com/username/repository/main/Documentation/simulator_preview.png) *(Note: Replace with actual screenshot after upload)*

## 🚀 Features

- **Full Robot Simulation**: Realistic movement and sensor modeling for the iRobot Create.
- **Sensor Suite**: Includes Bumpers, Cliff sensors, Wall sensors (IR), Virtual Walls, Sonar, LIDAR, and Camera-based beacon tracking.
- **Custom Environments**: Create and edit map files with walls, beacons, and virtual walls using the built-in Map Maker GUI.
- **Manual & Autonomous Control**: Control the robot using keyboard/GUI buttons or interface with your own autonomous scripts.
- **Session Replay**: Record and replay simulation sessions for analysis.
- **Noise Modeling**: Configurable noise parameters for realistic sensor data.

## 🛠️ Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/iRobotCreateSimulatorToolbox-v2.git
   ```
2. **Open MATLAB** and navigate to the project directory.
3. **Add to Path**: Right-click the folder and select "Add to Path" > "Selected Folders and Subfolders".

## 📖 How to Use

### 1. Launch the Simulator
In the MATLAB Command Window, type:
```matlab
SimulatorGUI
```
This opens the main simulator interface. From here you can:
- **Load Map**: Load environment files (.txt).
- **Manual Control**: Use arrow keys (or W, A, S, D) to drive the robot.
- **Sensors**: View real-time sensor data in the command window.

### 2. Create Custom Maps
To design your own environment, run:
```matlab
MapMakerGUI
```
Use the drawing tools to place walls, dashed lines, beacons, and virtual walls. Save your map as a `.txt` file to use in the simulator.

### 3. Replay Sessions
To review previous runs, use:
```matlab
ReplayGUI
```

### 4. Custom Robot Configuration
You can define sensor noise and communication delays by creating/loading a configuration file. See the `Example Files` directory for templates.

## 📁 Project Structure

- `SimulatorGUI.m`: Main simulation engine and UI.
- `MapMakerGUI.m`: Environment design tool.
- `ReplayGUI.m`: Session playback tool.
- `CreateRobot.m`: Core robot class and physical modeling.
- `Documentation/`: Detailed user guides and function specifications (PDF).
- `Example Files/`: Sample maps and configurations.

## 📄 License

This project is released under the **BSD License**. See the [License.txt](Documentation/License.txt) file in the `Documentation` folder for details.

---
*Developed at Cornell University. Modified for enhanced UI and functionality.*
