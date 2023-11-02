# Obstacle Avoidance using Machine Learning

Welcome to the GitHub repository for the project on obstacle avoidance using machine learning! In this project, a solution that employs the power of machine learning and deep learning to enable a robot to navigate and avoid obstacles in its environment and reach predefined goal locations was developed, utilizing the monocular depth estimation technique with U-Net architecture. 

## Project Overview

## Introduction
The Obstacle Avoidance project leverages machine learning techniques to enable a Kobuki Turtlebot 2 to navigate its environment while avoiding obstacles and reaching predefined goal locations. This project combines monocular depth estimation, the SIFT algorithm, and careful floor data processing to achieve efficient and safe robot navigation.
<div align="center">
    <p><strong>Kobuki Turtlebot 2</strong></p>
    <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/c3b77f59-bca4-4732-9e47-f11abe129b70">
</div>

## Key Components

### Monocular Depth Estimation
A monocular depth estimation model trained on the NYU Depth V2 dataset, is employed to generate depth images from a single camera. These depth images provide crucial information about the environment's spatial structure, enabling the robot to perceive obstacles and distances effectively.

### UNet Architecture with DenseNet Encoder
The core of the depth estimation model is based on a UNet architecture with a DenseNet model serving as the encoder. This combination enhances the model's ability to extract features and make accurate depth predictions.

**Input image and Predicted depth image:**

<div align="center">
    <p><strong>Input image and Predicted depth image</strong></p>
    <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/4455cb8f-8188-4b5c-b5c0-5192aeb89f09" alt="Input image and Predicted depth image">
</div>

### MobileNet Encoder
An alternative encoder based on MobileNet architecture was also considered and tested. While not the primary choice, it provides faster inference time but with a lower accuracy.

### SIFT Algorithm
SIFT (Scale-Invariant Feature Transform) is used to detect the goal by identifying distinctive image features that are invariant to scale, rotation, and lighting changes. These features, or keypoints, are matched between the robot's camera image and the reference goal image. RANSAC is then applied to improve accuracy and determine the goal's location and orientation. After goal detection, a rectangle is drawn around it using perspective transform to find its center position. The goal's center position is used to correct the robot's position by calculating steering angles for goal pursuit while avoiding any obstacles in between.

<div align="center">
    <p><strong>Goal Identification Using SIFT</strong></p>
    <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/cffd66a2-d6b4-481f-8931-94837eba3dcd" alt="Goal Recognition">
</div>

### Floor Data Processing
To address potential challenges posed by the floor, the project includes a floor data processing step. The floor data is marked away by calculating normals of the image, ensuring that any surface appearing as floors, mats, or small objects that are not any obstacle for the robot are marked out.

<div align="center">
    <p><strong>Robot's Depth View</strong></p>
    <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/03ce415b-540e-4301-811e-419bd6208640" alt="Floor Data Processing">
</div>
<div align="center">
    <p><strong>Floor Data Marked Out by Calculating Normals</strong></p>
    <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/e8c867b0-97e2-4c89-8dd9-14c078f0b38b" alt="Floor Data Processing">
</div>

### Models

1. **DenseNet-based Monocular Depth Estimation Model**: In the [`Dense_net_model_training.ipynb`](Dense_net_model_training.ipynb), provides a comprehensive guide on training a U-Net architecture for monocular depth estimation using the DenseNet model as the encoder. This model learns to perceive depth from a single input image and has demonstrated higher accuracy of 97% on validation data and 86% on testing data.

2. **MobileNet-based Monocular Depth Estimation Model**: The [`Mobile_net_model_training.ipynb`](Mobile_net_model_training.ipynb) demonstrates the training process for another U-Net architecture, this time utilizing the MobileNet model as the encoder. While this model achieves slightly lower accuracy compared to the DenseNet model (94% on validation data and 80% on testing data), it offers a significant advantage in terms of faster inference times. This can be especially beneficial when hardware limitations are a concern.

### Scripts

1. **obstacle_avoidance.py**: This Python script showcases the practical implementation of the obstacle avoidance approach on a Kobuki Turtlebot 2. The script integrates the trained depth estimation model to enable the robot to autonomously navigate its environment while avoiding obstacles.
<div align="center">
  <p></p>
</div>
<p align="center">
  <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/c344830a-358d-4879-ad16-4b14cb07c51f" alt="Obstacle Avoidance">
</p>
<div align="center">
  <p></p>
</div>

2. **goal_final.py**: Building upon the obstacle avoidance concept, this script takes the project a step further. It incorporates the SIFT algorithm to establish a predefined goal and guides the robot towards it. The script showcases how the machine learning-based approach can be extended to incorporate higher-level decision-making processes.
<div align="center">
  <p></p>
</div>
<p align="center">
  <img src="https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/0b109ff9-0f5d-4b7c-ba58-be23c0575b78" alt="Goal Detection">
</p>
<div align="center">
  <p></p>
</div>

### Additional Files

- [`Project_Report.pdf`](Project_Report.pdf): A detailed report outlining the project's goals, methodology, results, and insights. It provides a comprehensive overview of this approach and its outcomes.
- [`Demonstration.md`](Demonstration.md): This directory contains video demonstration links for the two different implementations.

## Getting Started

If you're interested in using this project for your own applications, follow these steps:

1. **Data Preparation**: Download and preprocess the [NYU Depth V2 dataset](https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html) as described in the model training notebooks ([`Dense_net_model_training.ipynb`](Dense_net_model_training.ipynb) and [`Mobile_net_model_training.ipynb`](Mobile_net_model_training.ipynb)).

2. **Model Training**: Use the provided notebooks to train your preferred depth estimation model (DenseNet or MobileNet).
   
3. **ROS Installation**: Install ROS on your system, follow the official installation instructions provided in the [ROS documentation](https://wiki.ros.org/noetic/Installation/Ubuntu).

4. **Robot Deployment**: Utilize the [`obstacle_avoidance.py`](obstacle_avoidance.py) script to deploy the trained model on a Kobuki Turtlebot 2 and observe its obstacle avoidance capabilities.

5. **Advanced Goal Navigation**: For more advanced navigation with goal setting, refer to the [`goal_final.py`](goal_final.py) script, which combines obstacle avoidance with the SIFT algorithm for goal-oriented movement.

## Conclusion

This project demonstrates the integration of machine learning techniques with robotics to achieve effective obstacle avoidance and goal-driven navigation. Hope this repository serves as a valuable resource for those interested in applying similar approaches to their own robotic systems. Feel free to explore the code, experiment with different settings, and adapt the solution to your specific needs.

If you have any questions or feedback, please don't hesitate to get in touch [Email](mailto:shreygupta0509@gmail.com). Happy coding!
