import numpy as np

def distance(p1, p2):
    return np.linalg.norm(p2 - p1)

def calculate_joint_angles(joint_positions):
    angles = []
    for i in range(len(joint_positions) - 1):
        vec1 = joint_positions[i + 1] - joint_positions[i]
        angle = np.arctan2(vec1[1], vec1[0])
        angles.append(angle)
    return angles

def fabrik(target, joint_positions, joint_lengths, tolerance=0.01, max_iterations=1000):
    num_joints = len(joint_positions)
    link_lengths = joint_lengths
    total_length = sum(link_lengths)
    if distance(joint_positions[0], target) > total_length:
        # Target is unreachable
        return None

    iterations = 0
    current_target = np.copy(target)
    initial_distance = distance(joint_positions[0], target)
    while distance(joint_positions[-1], current_target) > tolerance and iterations < max_iterations:
        # Forward reaching
        joint_positions[-1] = np.copy(current_target)
        for i in range(num_joints - 2, -1, -1):
            link_direction = (joint_positions[i + 1] - joint_positions[i]) / link_lengths[i]
            joint_positions[i] = joint_positions[i + 1] - link_direction * link_lengths[i - 1]

        # Backward reaching
        joint_positions[0] = np.copy(joint_positions[0])
        for i in range(num_joints - 1):
            link_direction = (joint_positions[i + 1] - joint_positions[i]) / link_lengths[i]
            joint_positions[i + 1] = joint_positions[i] + link_direction * link_lengths[i]

        current_target = np.copy(target)
        iterations += 1

    final_distance = distance(joint_positions[0], target)
    if final_distance > initial_distance:
        # Target is unreachable
        return None

    joint_angles = calculate_joint_angles(joint_positions)
    return joint_positions, joint_angles

# Input for target position
target_x = float(input("Enter the x-coordinate of the target position: "))
target_y = float(input("Enter the y-coordinate of the target position: "))
target_z = float(input("Enter the z-coordinate of the target position: "))
target_position = np.array([target_x, target_y, target_z])

# Input for initial joint positions
num_joints = 4  # Number of joints for a 4-joint robot
initial_joint_positions = []
for i in range(num_joints):
    joint_x = float(input(f"Enter the x-coordinate of joint {i+1}: "))
    joint_y = float(input(f"Enter the y-coordinate of joint {i+1}: "))
    joint_z = float(input(f"Enter the z-coordinate of joint {i+1}: "))
    initial_joint_positions.append(np.array([joint_x, joint_y, joint_z]))

# Joint lengths
joint_lengths = [23, 15, 1]

# Call the FABRIK algorithm
result = fabrik(target_position, initial_joint_positions, joint_lengths)

# Process the result
if result is not None:
    joint_positions, joint_angles = result
    print("Joint positions:", joint_positions)
    print("Optimal joint angles (in radians):", joint_angles)
else:
    print("unreachable")
