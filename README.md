# Conduit Agent: Prototype Universal Driver

**Architectural Proof-of-Concept for Async Robot Orchestration**

### 1. The Mission
To architect a scalable, "Layer 2" middleware node capable of bridging the gap between stateless web orchestrators (Chalice/REST) and stateful, non-deterministic physical hardware (Fanuc, MiR, PLCs).

This project simulates a **"Universal Driver"** pattern:
* **The Orchestrator (Client):** Represents the FactoryOS control plane.
* **The Robot (Server):** Represents a hardware interface with variable latency.
* **The Protocol:** ROS 2 Actions (DDS) for robust, pre-emptible communication.

### 2. Architecture & Design Decisions

![ROS 2 Action Interface Diagram](https://design.ros2.org/img/actions/goal_state_machine.png)

#### Why Actions instead of Services?
In brownfield manufacturing, latency is a feature, not a bug. A Service call blocks the thread, which is dangerous when a palletizer takes 45 seconds to complete a cycle.
* **Async Pattern:** Used `ActionClient` to allow the Orchestrator to monitor multiple robots simultaneously without blocking.
* **Observability:** Implemented 1Hz Feedback loops (`progress_percentage`) to ensure the "FactoryOS" is never blind to the robot's state.

#### Safety-First: The "E-Stop" Logic
The "Happy Path" is easy. This architecture prioritizes the "Un-Happy Path."
* **Cancellation Support:** The robot node listens for `CancelGoal` requests during its execution loop.
* **Latency Check:** Cancellation signals propagate and arrest motion in <10ms (verified in logs).
* **State Integrity:** Distinguishes between `ABORTED` (System Failure) and `CANCELED` (Operator Intervention) for accurate fleet analytics.

#### "Day 2" Operations: Containerization
To eliminate dependency hell and match Conduit's on-prem deployment model:
* **Docker-Native:** The entire stack runs in an isolated container based on `ros:jazzy`.
* **Reproducibility:** No host-side ROS installation required.
* **Microservices:** Nodes run as isolated processes (verified via PID checks), launched via a single entry point (`factory.launch.py`).

### 3. Quick Start (Docker)

**1. Build the Microservice**
```bash
docker build -t conduit_dev .
```

**2. Run the Simulation (Auto-Launch)**
This mounts the local source code (hot-reloading) and executes the launch file.
```bash
docker run -it --rm \
  -v $(pwd)/src:/root/conduit_ws/src \
  conduit_dev \
  /bin/bash -c "colcon build && source install/setup.bash && ros2 launch conduit_agent factory.launch.py"
```

### 4. Verification Logs

**Scenario: Successful Async Execution**
```text
[orchestrator]: Sending mission MISSION_001...
[factory_robot]: Received goal request
[factory_robot]: Feedback: 10.0%
[orchestrator]: Received feedback: Status: WORKING, Progress: 10.0%
...
[orchestrator]: Result: Success=True, Status=COMPLETED
```

**Scenario: Emergency Cancellation (E-Stop)**
```text
[orchestrator]: Canceling mission after 3 seconds...
[factory_robot]: Received cancel request
[factory_robot]: Goal canceled
[orchestrator]: Result: Success=False, Status=CANCELED
```