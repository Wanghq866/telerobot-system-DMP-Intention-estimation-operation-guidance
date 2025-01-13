# Telerobot-system-DMP-Intention-estimation-operation-guidance
Manuscript： "A telerobotic shared control architecture for learning and generalizing skills in unstructured environments".

Direct teleoperation of robots in unstructured environments by non-experts often leads to low efficiency and increased risk. To this end, this paper proposes a shared control architecture where the robot can generalize demonstrations based on variable environments (start, obstacles, and goal positions) and infer user intent online to assist tasks safely and efficiently. First, the complex task is decomposed into unit actions, where the cubic-quintic-cubic Bezier curve is utilized to resolve the limitation of the classical dynamic movement primitives (DMP) algorithm in generalizing via-points. Gaussian process regression (GPR) was implemented to generalize expert demonstrations to varying environments, ensuring full generalizable trajectory in the subtask space. GPR further extrapolate simple demonstrations from a subspace to the entire task space, avoiding the need for numerous original demonstrations. Then, the online evolution of DMP is optimized: 1) an adaptive temporal scaling system is developed to synchronize evolution with human operations; 2) an intention prediction and expected evolution selection method based on operational input is proposed, achieving human-led operation guidance. Experiments validate the effectiveness of the generalization, obstacle avoidance, and intention prediction. Tests with non-experts reported the proposed architecture enhances efficiency, reduces collisions, and improves perceived safety.

# Folder Content Description
Folder "Leader device" contains the files of the haptic device system in ROS.

Folder "Follower device" contains the files of the 7-DoF robot device system in ROS.

File "A telerobotic shared control architecture for learning and generalizing skills in unstructured environments.mp4" is a video of the operation of the experimental verification process.
