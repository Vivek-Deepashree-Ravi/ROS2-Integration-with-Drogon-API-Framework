# ROS2 Integration with Drogon API Framework

## 🚀 Introduction
This project demonstrates an basic integration between **ROS2** (Robot Operating System 2) and **Drogon**, a high-performance, modern C++ web framework. By seamlessly connecting ROS2 nodes with web APIs, this solution enables both **publishing** and **subscribing** functionalities through a scalable and efficient interface.

## 🌟 Key Features
- **Real-time Communication:** Effortlessly expose and control ROS2 topics via RESTful APIs.
- **Publisher Node:** Accepts HTTP POST requests to publish messages to a ROS2 topic.
- **Subscriber Node:** Provides access to the latest messages from a ROS2 topic via an HTTP GET endpoint.
- **Thread-safe Multi-threading:** Ensures responsive handling of concurrent API requests.
- **Highly Scalable Design:** Easily extendable for more complex robot communication workflows.

## 🛠️ Architecture
- **Publisher API:** Handles incoming requests to publish messages to ROS2 topics.
- **Subscriber API:** Returns the latest received message from a ROS2 topic.
- **Thread-safe ROS2 Communication:** Separate execution threads for ROS2 spinning operations.

## 🚀 Usage Instructions
### Prerequisites
Ensure the following dependencies are installed:
- **ROS2 Humble**
- **Drogon Framework**

#### Install Drogon Framework
To install Drogon on Ubuntu:
```bash
sudo apt update
sudo apt install cmake g++ uuid-dev openssl libssl-dev zlib1g-dev
cd ~
git clone https://github.com/drogonframework/drogon.git
cd drogon
mkdir build
cd build
cmake ..
make
sudo make install
```

### Clone the Repository
```bash
git clone https://github.com/Vivek-Deepashree-Ravi/ROS2-Integration-with-Drogon-API-Framework.git
cd ros2-drogon-api
```

### Build the Project
```bash
colcon build
```

### Running the Nodes
1. Start the **Publisher Node**:
   ```bash
   ros2 run api_node api_node_publisher
   ```

2. Start the **Subscriber Node**:
   ```bash
   ros2 run api_node api_node_subscriber
   ```

### API Usage
- **Publish a Message**
  - **Endpoint:** `POST /publish`
  - **Body:** `{ "message": "Hello, ROS2!" }`

- **Get Latest Message**
  - **Endpoint:** `GET /latest_message`

### Example Requests
#### Publish a Message
```bash
curl -X POST -H "Content-Type: application/json" -d '{"message": "Hello, ROS2!"}' http://localhost:8080/publish
```
#### Retrieve Latest Message
```bash
curl -X GET http://localhost:8081/latest_message
```

## 🤖 Applications
- Remote robot control using web APIs.
- Bridging robotics systems with external cloud applications.
- IoT integration for real-time monitoring.

## 🏆 Highlights
This project may be one of the **first demonstrations** of integrating **ROS2 with Drogon** to unlock advanced robotic communication capabilities. With the combination of ROS2's robotics power and Drogon's high-performance API design, it lays a foundation for exciting new possibilities in automation.




