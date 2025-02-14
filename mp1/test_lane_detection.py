import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
from utils.lane_detector import LaneDetector
from models.enet import ENet
import os

# Define dataset and checkpoint paths
DATASET_PATH = "/opt/data/TUSimple/test_set"
CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best.pth"  # Path to the trained model checkpoint


# Function to load the ENet model
def load_enet_model(checkpoint_path, device="cuda"):
    enet_model = ENet(binary_seg=2, embedding_dim=4).to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    enet_model.load_state_dict(checkpoint['model_state_dict'])
    enet_model.eval()
    return enet_model


def perspective_transform(image):
    """
    Transform an image into a bird's eye view.
        1. Calculate the image height and width.
        2. Define source points on the original image and corresponding destination points.
        3. Compute the perspective transform matrix using cv2.getPerspectiveTransform.
        4. Warp the original image using cv2.warpPerspective to get the transformed output.
    """

    ####################### TODO: Your code starts Here #######################

    HORIZON_COEF = 0.32  # CRITICAL hyperparameter
    height, width = 0, 0
    if len(image.shape) == 2:
        height, width = image.shape
    elif len(image.shape) == 3:
        height, width, channels = image.shape
    horizon = int(HORIZON_COEF * height)
    y_top = int(0.45 * height)
    y_bot = int(0.85 * height)  # we sacrifice a bit of the bottom to get a wider frame
    halfwidth = width // 2
    # Calculate intersection parameters
    # Area is a trapezoid defined by clip_top and clip_bottom, with side diagonals converging at the horizon
    # 
    # --------------------+----------------- y = horizon
    #                   /  x = halfwidth
    # -----------------*-------------------- y = clip_top
    #                /  x = halfwidth*(1- (top - horizon)/(bottom - horizon)) calculate via proportionality
    #              /
    #      ...   / ... all the way down to x = 0
    #   /
    # *------------------------------------- y = clip_bottom
    # x = 0

    t = (y_top - horizon) / (y_bot - horizon)
    x1 = int(halfwidth * (1 - t))
    x2 = int(width - x1)  # mirror x1 to ge x2

    # define source using computed coordinates, in ccw order
    p1 = (0, y_bot)
    p2 = (width - 1, y_bot)
    p3 = (x2, y_top)
    p4 = (x1, y_top)
    src = np.float32([p1, p2, p3, p4])

    # define destination using existing img size, in same order
    dst = np.float32([
        (0, height - 1),
        (width - 1, height - 1),
        (width - 1, 0),
        (0, 0)
    ])

    P = cv2.getPerspectiveTransform(src, dst)
    transformed_image = cv2.warpPerspective(image, P, (width, height), flags=cv2.INTER_CUBIC)

    ####################### TODO: Your code ends Here #######################

    return transformed_image


# Function to visualize lane predictions for multiple images in a single row
def visualize_lanes_row(images, instances_maps, alpha=0.7):
    """
    Visualize lane predictions for multiple images in a single row
    For each image:
        1. Resize it to 512 x 256 for consistent visualization.
        2. Apply perspective transform to both the original image and its instance map.
        3. Overlay the instance map to a plot with the corresponding original image using a specified alpha value.
    """

    num_images = len(images)
    fig, axes = plt.subplots(1, num_images, figsize=(15, 5))

    ####################### TODO: Your code starts Here #######################

    for i in range(num_images):
        image = images[i]
        instances_map = instances_maps[i]

        # Resize image to 512 x 256
        image = cv2.resize(image, (512, 256))
        instances_map = cv2.resize(instances_map, (512, 256))

        # Apply perspective transform to the original image and its instance map
        transformed_image = perspective_transform(image)
        transformed_instances_map = perspective_transform(instances_map)
        print(transformed_instances_map)
        # print(f"Image shape: {transformed_instances_map.shape}")
        transformed_instances_map = np.stack([transformed_instances_map,
                                              np.zeros_like(transformed_instances_map),
                                              np.zeros_like(transformed_instances_map)],
                                             axis=-1)
        # transformed_instances_map = np.pad(transformed_instances_map, 2, mode='empty', axis=2)
        # transformed_instances_map = transformed_instances_map.astype(np.uint8)
        normalized_instance_map = cv2.normalize(transformed_instances_map, None, 0, 255, cv2.NORM_MINMAX)
        transformed_instances_map = normalized_instance_map.astype(np.uint8)

        overlay = cv2.addWeighted(transformed_image, alpha, transformed_instances_map, 1 - alpha, 0)

        # Plot the overlay
        axes[i].imshow(overlay)
        axes[i].axis("off")
        axes[i].set_title(f"Image {i + 1}")

    ####################### TODO: Your code ends Here #######################

    plt.tight_layout()
    plt.show()


def main():
    # Initialize device and model
    device = "cuda" if torch.cuda.is_available() else "cpu"
    enet_model = load_enet_model(CHECKPOINT_PATH, device)
    lane_predictor = LaneDetector(enet_model, device=device)

    # List of test image paths
    sub_paths = [
        "clips/0530/1492626047222176976_0/20.jpg",
        "clips/0530/1492626286076989589_0/20.jpg",
        "clips/0531/1492626674406553912/20.jpg",
        "clips/0601/1494452381594376146/20.jpg",
        "clips/0601/1494452431571697487/20.jpg"
    ]
    test_image_paths = [os.path.join(DATASET_PATH, sub_path) for sub_path in sub_paths]

    # Load and process images
    images = []
    instances_maps = []

    for path in test_image_paths:
        image = cv2.imread(path)
        if image is None:
            print(f"Error: Unable to load image at {path}")
            continue

        print(f"Processing image: {path}")
        instances_map = lane_predictor(image)
        images.append(image)
        instances_maps.append(instances_map)

    # Visualize all lane predictions in a single row
    if images and instances_maps:
        visualize_lanes_row(images, instances_maps)


if __name__ == "__main__":
    main()
