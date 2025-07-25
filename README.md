🛰️ Autonomous Drone Navigation System
An intelligent drone system capable of autonomous obstacle detection, path planning, and real-time flight control using YOLO, infrared sensors, and MAVLink protocol.

🚀 Features
🧠 Modular Architecture: Clean separation of Perception, Decision, and Control layers

👁️ YOLOv3 Object Detection for obstacle awareness

📡 Infrared Sensors for distance estimation

🧭 Path Planning using a simplified Hybrid A* algorithm

🎮 Real-Time Flight Commands via MAVLink interface

🧩 Project Structure
bash
Copy
Edit
📂 AutonomousDrone/
├── drone.py                # Main implementation file (all classes)
├── yolov3.cfg              # YOLO configuration file
├── yolov3.weights          # YOLO pre-trained weights
├── test_image.jpg          # Test image for simulation
├── README.md               # Project documentation


🛠️ Setup Instructions
Clone the repository

bash
Copy
Edit
git clone https://github.com/yourusername/AutonomousDrone.git
cd AutonomousDrone
Install dependencies

bash
Copy
Edit
pip install opencv-python dronekit pymavlink
Add YOLOv3 files

Download yolov3.weights from the official YOLO site

Place it in the root directory along with yolov3.cfg

Run the project

bash
Copy
Edit
python drone.py
⚠️ Ensure your MAVLink-compatible flight simulator (e.g., SITL) is running and connected at 127.0.0.1:14550.


📐 System Architecture
text
Copy
Edit
[ Camera + IR Sensor ]
         ↓
  ┌────────────────────┐
  │  Perception Layer  │ ← YOLOv3 + IR Distance
  └────────────────────┘
         ↓
  ┌────────────────────┐
  │  Decision Layer     │ ← A* Path Planning
  └────────────────────┘
         ↓
  ┌────────────────────┐
  │  Control Layer      │ ← MAVLink Commands
  └────────────────────┘
         ↓
       [ Drone ]

       

🧪 To-Do / Future Work
 Add real-time camera input

 Integrate GPS-based waypoint navigation

 Extend Hybrid A* for 3D environments

 Implement obstacle avoidance during flight
 

🤝 Contributing
Contributions are welcome! Please:

Fork this repo

Create a feature branch

Submit a pull request with a clear description

