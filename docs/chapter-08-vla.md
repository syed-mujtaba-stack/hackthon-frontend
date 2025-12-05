---
sidebar_position: 9
---

# Chapter 8: Vision-Language-Action (VLA) Models

## Learning Objectives

- Understand the VLA paradigm shift in robotics
- Explore RT-1, RT-2, and other VLA architectures
- Implement end-to-end vision-to-action pipelines
- Fine-tune VLA models for custom tasks
- Deploy VLA models on humanoid robots
- Understand the role of foundation models in robotics

## The VLA Revolution

Traditional robotics pipelines are **modular**:

```
Camera → Object Detection → Pose Estimation → Motion Planning → Control
```

**Problems**:
- Each module has errors that compound
- Requires manual engineering for each task
- Doesn't generalize to new scenarios

**VLA Models** are **end-to-end**:

```
Camera Image + Language Command → Motor Commands
```

**Advantages**:
- Single neural network (no error propagation)
- Learns from data (less manual engineering)
- Generalizes via pre-training on large datasets

## Foundation Models in Robotics

VLA models leverage **foundation models** pre-trained on internet-scale data:

| Model | Pre-training Data | Robotics Application |
|-------|-------------------|----------------------|
| **CLIP** | 400M image-text pairs | Visual grounding |
| **GPT-4** | Trillions of tokens | Task planning, reasoning |
| **SAM** | 1B masks | Object segmentation |
| **DINOv2** | 142M images | Visual features |

**Key Insight**: Pre-training on diverse data enables **zero-shot generalization** to new objects and tasks.

## RT-1: Robotics Transformer

**RT-1** (Google DeepMind, 2022) was the first large-scale VLA model.

### Architecture

```
Input: RGB Image (320x256) + Language Command ("pick up the apple")
       ↓
    Vision Encoder (EfficientNet)
       ↓
    Language Encoder (Universal Sentence Encoder)
       ↓
    Transformer (8 layers)
       ↓
    Action Tokens (7-DOF arm + gripper)
```

### Training Data

- **130,000 episodes** from 13 robots
- **700 tasks** (pick, place, push, open drawer, etc.)
- Real-world data (no simulation)

### Performance

- **97% success** on seen tasks
- **76% success** on novel objects (zero-shot)

**Example**:

```
Command: "Pick up the blue block"
RT-1 Output: [x, y, z, roll, pitch, yaw, gripper_open]
```

## RT-2: Vision-Language-Action with VLMs

**RT-2** (2023) improves RT-1 by using **Vision-Language Models (VLMs)** like PaLM-E.

### Key Innovation

Pre-train on **web-scale vision-language data**, then fine-tune on robot data.

**Training Pipeline**:

1. **Pre-training**: PaLM-E on 1B image-text pairs
2. **Co-fine-tuning**: Mix web data + robot data
3. **Action Head**: Add output layer for motor commands

### Results

- **Emergent capabilities**: Can follow complex instructions ("move the apple to the left of the banana")
- **Reasoning**: "Which object is the heaviest?" → picks the correct one
- **Generalization**: 3x better than RT-1 on novel objects

## Implementing a Simple VLA Model

### Step 1: Data Collection

Collect tuples of `(image, language, action)`:

```python
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class DataCollector:
    def __init__(self):
        self.bridge = CvBridge()
        self.data = []
    
    def collect(self, image_msg, cmd_vel_msg, language_command):
        # Convert ROS image to numpy
        image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        
        # Extract action
        action = [
            cmd_vel_msg.linear.x,
            cmd_vel_msg.linear.y,
            cmd_vel_msg.angular.z
        ]
        
        # Store
        self.data.append({
            'image': image,
            'language': language_command,
            'action': action
        })
```

### Step 2: Model Architecture (PyTorch)

```python
import torch
import torch.nn as nn
from transformers import CLIPModel, CLIPProcessor

class SimpleVLA(nn.Module):
    def __init__(self, action_dim=3):
        super().__init__()
        
        # Vision encoder (CLIP)
        self.clip = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        
        # Freeze CLIP (transfer learning)
        for param in self.clip.parameters():
            param.requires_grad = False
        
        # Action head
        self.action_head = nn.Sequential(
            nn.Linear(512, 256),  # CLIP embedding size = 512
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(256, action_dim)
        )
    
    def forward(self, images, texts):
        # Get CLIP embeddings
        inputs = self.processor(
            text=texts, 
            images=images, 
            return_tensors="pt", 
            padding=True
        )
        
        outputs = self.clip(**inputs)
        
        # Combine vision and language features
        image_embeds = outputs.image_embeds
        text_embeds = outputs.text_embeds
        combined = image_embeds + text_embeds  # Simple fusion
        
        # Predict actions
        actions = self.action_head(combined)
        return actions
```

### Step 3: Training Loop

```python
import torch.optim as optim

model = SimpleVLA(action_dim=3)
optimizer = optim.Adam(model.parameters(), lr=1e-4)
criterion = nn.MSELoss()

for epoch in range(100):
    for batch in dataloader:
        images = batch['image']
        texts = batch['language']
        actions_gt = batch['action']
        
        # Forward pass
        actions_pred = model(images, texts)
        
        # Loss
        loss = criterion(actions_pred, actions_gt)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        print(f"Epoch {epoch}, Loss: {loss.item()}")
```

### Step 4: Inference on Robot

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        self.model = SimpleVLA()
        self.model.load_state_dict(torch.load('vla_model.pth'))
        self.model.eval()
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_command = "move forward"
    
    def image_callback(self, msg):
        # Convert image
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        
        # Predict action
        with torch.no_grad():
            action = self.model([image], [self.current_command])
        
        # Publish
        cmd = Twist()
        cmd.linear.x = float(action[0, 0])
        cmd.linear.y = float(action[0, 1])
        cmd.angular.z = float(action[0, 2])
        self.cmd_pub.publish(cmd)
```

## Open-Source VLA Models

### OpenVLA (Stanford, 2024)

**OpenVLA** is an open-source 7B parameter VLA model.

**Installation**:

```bash
pip install openvla
```

**Usage**:

```python
from openvla import OpenVLA

model = OpenVLA.from_pretrained("openvla/openvla-7b")

# Inference
action = model.predict(
    image=camera_image,
    instruction="pick up the red cup"
)
```

### Octo (UC Berkeley, 2024)

**Octo** is a generalist robot policy trained on 800k robot trajectories.

```python
from octo.model import OctoModel

model = OctoModel.load_pretrained("hf://rail-berkeley/octo-base")

action = model.sample_actions(
    observation={"image": image},
    task={"language_instruction": "open the drawer"}
)
```

## Challenges and Limitations

### 1. Data Hunger

VLA models require **millions** of robot interactions.

**Solutions**:
- **Simulation**: Generate data in Isaac Sim
- **Teleoperation**: Human demonstrations
- **Data Sharing**: Open-source datasets (Open X-Embodiment)

### 2. Safety

End-to-end models are **black boxes** → hard to guarantee safety.

**Solutions**:
- **Residual policies**: VLA + classical safety controller
- **Constrained action spaces**: Limit dangerous actions
- **Human-in-the-loop**: Require approval for critical actions

### 3. Computational Cost

Large VLA models (7B+ parameters) are slow on edge devices.

**Solutions**:
- **Model quantization**: INT8, INT4 precision
- **Knowledge distillation**: Train smaller student model
- **Hardware acceleration**: NVIDIA Jetson Orin, TPUs

## Lab Exercise: VLA for Humanoid Navigation

### Objective
Train a simple VLA model to navigate based on language commands.

### Step 1: Collect Data

```bash
ros2 run data_collector collect_nav_data.py
```

**Commands**:
- "move forward"
- "turn left"
- "turn right"
- "stop"

### Step 2: Train Model

```bash
python train_vla.py --data nav_data.pkl --epochs 50
```

### Step 3: Deploy

```bash
ros2 run vla_controller vla_nav_node
```

**Test**:

```bash
ros2 topic pub /vla/command std_msgs/String "data: 'turn left'"
```

## Quiz

1. **What is the main advantage of VLA models over modular pipelines?**
   - A) Faster computation
   - B) End-to-end learning reduces error propagation
   - C) Easier to debug
   - D) Requires less data
   
   **Answer: B**

2. **Which foundation model is used for visual grounding in VLA?**
   - A) GPT-4
   - B) BERT
   - C) CLIP
   - D) ResNet
   
   **Answer: C**

3. **What was RT-2's key innovation over RT-1?**
   - A) Larger robot arm
   - B) Pre-training on web-scale vision-language data
   - C) Faster inference
   - D) Smaller model size
   
   **Answer: B**

4. **Why are VLA models considered "black boxes"?**
   - A) They run on GPUs
   - B) End-to-end learning makes it hard to interpret decisions
   - C) They use encryption
   - D) They don't have source code
   
   **Answer: B**

## Summary

In this chapter, we explored Vision-Language-Action (VLA) models, the cutting-edge approach that unifies perception, reasoning, and control in a single neural network. We studied RT-1, RT-2, and open-source models like OpenVLA and Octo. We implemented a simple VLA model using CLIP and deployed it on a simulated humanoid. VLA models represent the future of robotics, enabling generalization and natural language control.

**Next Chapter**: We'll study control theory and balance for bipedal humanoid robots.
