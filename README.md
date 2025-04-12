# 🤖 Webots Reinforcement Learning Project employing E-PUCK Robot (Q-Learning)

This project uses **Q-learning** to train a Webots e-puck robot to navigate autonomously to a target while avoiding obstacles. Training and inference are separated into two controller files:

- `training.py`: Learns optimal actions using Q-learning
- `inference.py`: Runs the robot using the learned policy

---

## 🧠 Core Features

- **Q-learning** with exploration/exploitation (ε-greedy)
- Stores knowledge in a persistent `q_table.json`
- Records best path to target (`learned_path.json`)
- Real-time logging of each training episode
- Obstacle detection and avoidance logic
- Simple discrete state-action representation

---

## 📁 File Structure

| File               | Purpose                                           |
|--------------------|---------------------------------------------------|
| `training.py`      | Controller to train the robot using Q-learning    |
| `inference.py`     | Controller to deploy the learned policy           |
| `q_table.json`     | Learned Q-values (generated during training)      |
| `learned_path.json`| Best path recorded during training                |
| `training_output_*.log` | Training logs with detailed reward info     |

---

## 🛠️ Setup Instructions

1. Open your Webots world containing the **e-puck robot**.
2. Attach the following devices to the robot:
   - `left wheel motor`, `right wheel motor`
   - `gps`
   - Distance sensors: `ps0` to `ps7`
3. Set the robot controller to:
   - `training.py` (for training)
   - `inference.py` (for testing/inference)
4. Run the simulation.

---

## ⚙️ Parameters

| Parameter      | Value       | Description                          |
|----------------|-------------|--------------------------------------|
| `alpha`        | 0.1         | Learning rate                        |
| `gamma`        | 0.9         | Discount factor                      |
| `epsilon`      | 0.2         | Exploration rate                     |
| `max_steps`    | 5000        | Max steps per episode                |
| `target`       | (-0.5, 0.3) | Goal position                        |
| `threshold`    | 0.5         | Success radius from goal             |

---

## 🧪 Running Modes

### 🚀 Training

Set the controller to `training.py` to start the learning process.

```bash
# Logs will be saved to:
training_output_YYYYMMDD_HHMMSS.log
# and Q-table will be updated in:
q_table.json
```

---

## 👩‍💻 Developers Information

Developed by **Sheema Firdous [https://www.linkedin.com/in/sheema-firdous-67b9b8181/]**  
as part of the **Cognitive Systems and Robotics** module assessment  at **Sheffield Hallam University**

Supervised by [Dr. Samuele Vinanzi](https://www.linkedin.com/in/samuelevinanzi/)

This project demonstrates the practical application of reinforcement learning (Q-learning) in autonomous robotics using Webots and Python.
