import cv2
import os
import argparse

def create_video_from_frames(image_folder, output_video, fps=20):
    # Get list of images in the folder, sorted by name
    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png") or img.endswith(".jpg")])

    if not images:
        print("No images found in the folder.")
        return

    # Read the first image to get the size
    first_image_path = os.path.join(image_folder, images[0])
    frame = cv2.imread(first_image_path)
    height, width, layers = frame.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Save as MP4 file
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for image in images:
        image_path = os.path.join(image_folder, image)
        frame = cv2.imread(image_path)

        if frame is None:
            print(f"Error reading image {image_path}, skipping.")
            continue

        print('write to video {}'.format(image))
        video.write(frame)

    # Release the video writer
    video.release()
    print(f"Video created successfully: {output_video}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create a video from a sequence of images.")
    parser.add_argument('--path', type=str, required=True, help="Path to the folder containing image frames.")
    parser.add_argument('--output', type=str, default="output_video.mp4", help="Output video file name.")
    parser.add_argument('--fps', type=int, default=20, help="Frames per second for the output video.")

    args = parser.parse_args()
    
    create_video_from_frames(args.path, args.output, fps=args.fps)
