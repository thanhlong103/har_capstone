import cv2
import numpy as np
import random
import os
import argparse
import glob

# ---------- Fixed Parameter Generation ----------

def generate_augmentation_params():
    """
    Generate a dictionary of fixed transformation parameters to be applied
    to all frames of one augmented video.
    Each transformation is applied with a probability of 50%.
    """
    params = {}

    # Horizontal flip: True if selected, False otherwise.
    params['horizontal_flip'] = True if random.random() < 0.5 else False

    # Rotation: choose an angle between -15 and 15 degrees or None.
    params['rotation_angle'] = random.uniform(-15, 15) if random.random() < 0.5 else None

    # Brightness: choose a factor between 0.7 and 1.3 or None.
    params['brightness_factor'] = random.uniform(0.7, 1.3) if random.random() < 0.5 else None

    # Contrast: choose a factor between 0.7 and 1.3 or None.
    params['contrast_factor'] = random.uniform(0.7, 1.3) if random.random() < 0.5 else None

    # Gaussian noise: tuple (mean, sigma) or None.
    params['gaussian_noise'] = (0, 10) if random.random() < 0.5 else None

    # Gaussian blur: choose an odd kernel size between 3 and 7 or None.
    if random.random() < 0.5:
        possible_ksizes = [k for k in range(3, 8) if k % 2 == 1]
        params['blur_kernel'] = random.choice(possible_ksizes)
    else:
        params['blur_kernel'] = None

    # Saturation adjustment: choose a factor between 0.7 and 1.3 or None.
    params['saturation_factor'] = random.uniform(0.7, 1.3) if random.random() < 0.5 else None

    # Scaling: choose a scale factor between 0.8 and 1.2 or None.
    params['scaling'] = random.uniform(0.8, 1.2) if random.random() < 0.5 else None

    # Translation: choose (tx, ty) where each is between -10 and 10 pixels or None.
    if random.random() < 0.5:
        tx = random.randint(-10, 10)
        ty = random.randint(-10, 10)
        params['translation'] = (tx, ty)
    else:
        params['translation'] = None

    return params

# ---------- Apply Fixed Transformations ----------

def apply_transformations(frame, params):
    """
    Apply the fixed set of transformations (if selected) to the given frame.
    """
    # Horizontal flip
    if params.get('horizontal_flip'):
        frame = cv2.flip(frame, 1)

    # Rotation
    angle = params.get('rotation_angle')
    if angle is not None:
        (h, w) = frame.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        frame = cv2.warpAffine(frame, M, (w, h), borderMode=cv2.BORDER_REFLECT)

    # Brightness adjustment
    brightness_factor = params.get('brightness_factor')
    if brightness_factor is not None:
        frame = cv2.convertScaleAbs(frame, alpha=brightness_factor, beta=0)

    # Contrast adjustment
    contrast_factor = params.get('contrast_factor')
    if contrast_factor is not None:
        # Adjust contrast around the mean value.
        mean = np.mean(frame)
        frame = cv2.convertScaleAbs(frame, alpha=contrast_factor, beta=-(mean * (contrast_factor - 1)))

    # Gaussian noise
    gaussian_noise = params.get('gaussian_noise')
    if gaussian_noise is not None:
        mean, sigma = gaussian_noise
        noise = np.random.normal(mean, sigma, frame.shape).astype(np.int16)
        frame = frame.astype(np.int16) + noise
        frame = np.clip(frame, 0, 255).astype(np.uint8)

    # Gaussian blur
    blur_kernel = params.get('blur_kernel')
    if blur_kernel is not None:
        frame = cv2.GaussianBlur(frame, (blur_kernel, blur_kernel), 0)

    # Saturation adjustment
    saturation_factor = params.get('saturation_factor')
    if saturation_factor is not None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 1] *= saturation_factor
        hsv[..., 1] = np.clip(hsv[..., 1], 0, 255)
        frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    # Scaling
    scale = params.get('scaling')
    if scale is not None:
        (h, w) = frame.shape[:2]
        scaled = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        new_h, new_w = scaled.shape[:2]
        # If scaled image is larger, center-crop it; if smaller, pad it.
        if new_h > h or new_w > w:
            start_y = (new_h - h) // 2
            start_x = (new_w - w) // 2
            frame = scaled[start_y:start_y+h, start_x:start_x+w]
        else:
            pad_y = (h - new_h) // 2
            pad_x = (w - new_w) // 2
            frame = cv2.copyMakeBorder(scaled, pad_y, h - new_h - pad_y, pad_x, w - new_w - pad_x, cv2.BORDER_REFLECT)

    # Translation
    translation = params.get('translation')
    if translation is not None:
        tx, ty = translation
        (h, w) = frame.shape[:2]
        M = np.float32([[1, 0, tx], [0, 1, ty]])
        frame = cv2.warpAffine(frame, M, (w, h), borderMode=cv2.BORDER_REFLECT)

    return frame

# ---------- Video Processing Functions ----------

def process_single_video(input_path, output_path, params):
    """
    Process one video file by applying the fixed transformations (defined in params)
    to every frame, and save the augmented video to output_path.
    """
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"Error: Could not open video {input_path}")
        return

    # Get video properties.
    fps = cap.get(cv2.CAP_PROP_FPS)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    current_frame = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Apply the fixed transformations.
        augmented_frame = apply_transformations(frame, params)
        out.write(augmented_frame)

        current_frame += 1
        if current_frame % 50 == 0:
            print(f"    Processed {current_frame}/{frame_count} frames")

    cap.release()
    out.release()
    print(f"  Saved augmented video: {output_path}")

def process_videos_in_folder(folder, num_augmentations):
    """
    Find video files in the specified folder and, for each video, generate
    a fixed number of augmented versions, each with its own fixed transformation.
    """
    # Modify the extensions below as needed.
    video_extensions = ['*.mp4', '*.avi', '*.mov', '*.mkv']
    video_files = []
    for ext in video_extensions:
        video_files.extend(glob.glob(os.path.join(folder, ext)))
    if not video_files:
        print("No video files found in the specified folder.")
        return

    for video_path in video_files:
        print(f"\nProcessing video: {video_path}")
        base, ext = os.path.splitext(video_path)
        for i in range(1, num_augmentations + 1):
            suffix = f"-augment-{i:02d}{ext}"
            output_path = base + suffix
            print(f"  Creating augmentation {i}/{num_augmentations}")
            # Generate fixed transformation parameters for this augmentation.
            params = generate_augmentation_params()
            process_single_video(video_path, output_path, params)

# ---------- Main Function ----------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch Video Augmentation with Fixed Transformations")
    parser.add_argument("--folder", type=str, required=True,
                        help="Path to folder containing video files to augment")
    parser.add_argument("--num_augmentations", type=int, default=10,
                        help="Number of augmented versions to generate per video")
    args = parser.parse_args()

    process_videos_in_folder(args.folder, args.num_augmentations)
