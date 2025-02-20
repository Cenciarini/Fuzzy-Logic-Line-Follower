# 🤖 Robot Line Follower Simulation in CoppeliaSim

## 📌 Project Overview
This project simulates a **line-following robot** using **CoppeliaSim**. The robot moves along a line on the floor by interpreting sensor data and applying **fuzzy logic control** to adjust motor speeds dynamically.

## 🚀 Features
- **Integration with CoppeliaSim**: The robot is controlled through a Python script communicating via CoppeliaSim's API.
- **Fuzzy Logic Controller**: Adjusts motor speeds based on sensor readings to ensure smooth navigation.
- **Real-time Data Visualization**: Sensor values are plotted using Matplotlib for performance analysis.
- **Graphical Interface (PyQt5)**: Provides control buttons for starting/stopping the simulation.
- **Sensor-Based Decision Making**: Reads values from vision sensors to adjust motor speeds dynamically.

## 🛠️ Technologies Used
- **Python (PyQt5, NumPy, Matplotlib, scikit-fuzzy)**
- **CoppeliaSim Remote API**
- **Fuzzy Logic Control**
- **Real-time Data Processing**

## 📂 Project Structure
```
├── main.py                 # Main Python script to run the simulation
├── mobile.ttt              # CoppeliaSim scene file
├── qtUi.ui                 # PyQt5 UI file for the interface
├── remoteApi.dll           # Required DLL for CoppeliaSim API
├── sim.py                  # Python wrapper for CoppeliaSim API
├── simConst.py             # Constants for CoppeliaSim API
├── requirements.txt        # Dependencies for the project
├── README.md               # Project documentation
```


## 🔧 Installation & Setup
1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/Robot-Line-Follower.git
   cd Robot-Line-Follower
   ```
2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
3. **Open the CoppeliaSim scenario**:
   - Launch **CoppeliaSim**
   - Load `mobile.ttt`
4. **Run the simulation**:
   ```bash
   python main.py
   ```

## 🎯 How It Works
1. **Connects to CoppeliaSim** using the remote API.
2. **Reads sensor values** from three vision sensors (center, left, right).
3. **Computes position error** relative to the line.
4. **Applies fuzzy logic rules** to determine motor speeds.
5. **Sends commands to CoppeliaSim** to adjust motor velocities.
6. **Visualizes sensor data** in real-time.

## 📊 Fuzzy Logic Control
The fuzzy logic system is based on **error position** and outputs velocities for both motors:

| Error Position | Right Motor Speed | Left Motor Speed |
|---------------|-----------------|----------------|
| Large Negative | Very Slow | Very Fast |
| Medium Negative | Slow | Fast |
| Zero | Medium | Medium |
| Medium Positive | Fast | Slow |
| Large Positive | Very Fast | Very Slow |

## 📡 Real-Time Graphing
- Sensor values are plotted in real-time using **Matplotlib**.
- Helps analyze how the robot responds to line deviations.

## 🖥️ User Interface
- **Start/Stop Simulation** button.
- **Connect to CoppeliaSim** button.
- **Enable Data Plotting** button.

## 📜 License
This project is licensed under the **MIT License**.

---
✉️ **Author**: CENCIARINI Angel Gabriel & CABELLO Yasimel Joaquin
📍 **Developed for Facultad de Ciencias de la Alimentación, UNER**
