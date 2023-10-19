# Obstacle Avoidance using Machine Learning

Welcome to the GitHub repository for the project on obstacle avoidance using machine learning! In this project, a solution that employs the power of machine learning and deep learning to enable a robot to navigate and avoid obstacles in its environment was developed, utilizing the monocular depth estimation technique with U-Net architecture, trained on the NYU Depth V2 dataset. Additionally, two different encoder networks - DenseNet and MobileNet - to showcase the versatility were incorporated.

## Project Overview

This project is centered around two main models and accompanying scripts. Here's a breakdown of what you'll find in this repository:

### Models

1. **DenseNet-based Monocular Depth Estimation Model**: In the Dense_net_model_training.ipynb notebook, provides a comprehensive guide on training a U-Net architecture for monocular depth estimation using the DenseNet model as the encoder. This model learns to perceive depth from a single input image and has demonstrated higher accuracy in the experiments.

2. **MobileNet-based Monocular Depth Estimation Model**: The Mobile_net_model_training.ipynb notebook demonstrates the training process for another U-Net architecture, this time utilizing the MobileNet model as the encoder. While this model achieves slightly lower accuracy compared to the DenseNet model, it offers a significant advantage in terms of faster prediction times. This can be especially beneficial when hardware limitations are a concern.

### Scripts

1. **obstacle_avoidance.py**: This Python script showcases the practical implementation of the obstacle avoidance approach on a Kobuki Turtlebot 2. The script integrates the trained depth estimation model to enable the robot to autonomously navigate its environment while avoiding obstacles.
<p align="center">
  <img src="![Obstacle_Avoidance](https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/e7c40d9c-f05e-4fca-acc5-4e9628833c1b)" alt="Obstacle Avoidance">
</p>

2. **goal_final.py**: Building upon the obstacle avoidance concept, this script takes the project a step further. It incorporates the SIFT algorithm to establish a predefined goal and guides the robot towards it. The script showcases how the machine learning-based approach can be extended to incorporate higher-level decision-making processes.
<p align="center">
  <img src="![Goal Detection](https://github.com/Shrey5555/Obstacle-Avoidance-Using-Machine-Learning/assets/136813149/c29a0a1f-e1a6-44aa-b3ca-1d5c88a8db96)" alt="Goal Detection">
</p>

### Additional Files

- `Project_Report.pdf`: A detailed report outlining the project's goals, methodology, results, and insights. It provides a comprehensive overview of this approach and its outcomes.
- `Demonstration.md`: This directory contains video demonstration links for the two different implementations.

## Getting Started

If you're interested in using this project for your own applications, follow these steps:

1. **Data Preparation**: Download and preprocess the NYU Depth V2 dataset as described in the model training notebooks (`Dense_net_model_training.ipynb` and `Mobile_net_model_training.ipynb`).

2. **Model Training**: Use the provided notebooks to train your preferred depth estimation model (DenseNet or MobileNet).

3. **Robot Deployment**: Utilize the `obstacle_avoidance.py` script to deploy the trained model on a Kobuki Turtlebot 2 and observe its obstacle avoidance capabilities.

4. **Advanced Goal Navigation**: For more advanced navigation with goal setting, refer to the `goal_final.py` script, which combines obstacle avoidance with the SIFT algorithm for goal-oriented movement.

## Conclusion

This project demonstrates the integration of machine learning techniques with robotics to achieve effective obstacle avoidance and goal-driven navigation. Hope this repository serves as a valuable resource for those interested in applying similar approaches to their own robotic systems. Feel free to explore the code, experiment with different settings, and adapt the solution to your specific needs.

If you have any questions or feedback, please don't hesitate to get in touch [Email](mailto:shreygupta0509@gmail.com). Happy coding!
