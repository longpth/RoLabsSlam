import cv2
import numpy as np

class Frame:
    def __init__(self):
        self._keypoints = None
        self._descriptors = None
        self._orb = cv2.ORB_create()  # Initialize ORB detector

    def detectAndCompute(self, image):
        # Parameters for goodFeaturesToTrack
        maxCorners = 2000
        qualityLevel = 0.01
        minDistance = 10.0

        # Detect good features to track in the image
        goodFeatures = cv2.goodFeaturesToTrack(image, maxCorners, qualityLevel, minDistance)

        # Check if features are found
        if goodFeatures is not None:
            # Convert good features to keypoints
            goodKeypoints = [cv2.KeyPoint(x=pt[0][0], y=pt[0][1], size=1) for pt in goodFeatures]

            # Compute descriptors for the good keypoints
            self._keypoints, self._descriptors = self._orb.compute(image, goodKeypoints)
        else:
            self._keypoints = []
            self._descriptors = None

    def KeyPoints(self):
        return self._keypoints

    def Descriptors(self):
        return self._descriptors


def MatchKeyPoints(frame1, frame2):
    # Use BFMatcher to match descriptors
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    
    if frame1.Descriptors() is None or frame2.Descriptors() is None:
        print("No descriptors found in one or both frames")
        return []

    knnMatches = matcher.knnMatch(frame1.Descriptors(), frame2.Descriptors(), k=2)

    goodMatches = []
    idx1, idx2 = [], []
    idx1s, idx2s = set(), set()

    for matchPair in knnMatches:
        if len(matchPair) == 2 and matchPair[0].distance < 0.75 * matchPair[1].distance:
            p1 = frame1.KeyPoints()[matchPair[0].queryIdx]
            p2 = frame2.KeyPoints()[matchPair[0].trainIdx]

            # Be within ORB distance 32
            if matchPair[0].distance < 32:
                # Check for unique matches
                if matchPair[0].queryIdx not in idx1s and matchPair[0].trainIdx not in idx2s:
                    idx1.append(matchPair[0].queryIdx)
                    idx2.append(matchPair[0].trainIdx)
                    idx1s.add(matchPair[0].queryIdx)
                    idx2s.add(matchPair[0].trainIdx)
                    goodMatches.append(matchPair[0])

    return goodMatches


def draw_matches_vertically(img1, kp1, img2, kp2, good_matches):
    # Stack the two images vertically
    height1, width1 = img1.shape[:2]
    height2, width2 = img2.shape[:2]
    
    # Create an empty image to hold both images stacked vertically
    merged_image = np.zeros((height1 + height2, max(width1, width2), 3), dtype=np.uint8)
    
    # Place the two images in the merged_image
    merged_image[:height1, :width1] = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    merged_image[height1:height1 + height2, :width2] = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    
    # Draw lines between the matched keypoints
    for match in good_matches:
        # Get the keypoints from both images
        pt1 = tuple(map(int, kp1[match.queryIdx].pt))
        pt2 = tuple(map(int, kp2[match.trainIdx].pt))
        
        # Adjust pt2's y-coordinate to account for the vertical stacking
        pt2 = (pt2[0], pt2[1] + height1)

        # Draw the match line
        color = (0, 255, 0)  # Green color for the match line
        cv2.line(merged_image, pt1, pt2, color, 1)

        # Optionally, draw circles at the keypoints
        cv2.circle(merged_image, pt1, 5, (255, 0, 0), 1)
        cv2.circle(merged_image, pt2, 5, (255, 0, 0), 1)

    return merged_image

def main():
    # Initialize frames
    frame1 = Frame()
    frame2 = Frame()

    # Load images (grayscale)
    image1 = cv2.imread('.\\..\\Datasets\\000000.png', cv2.IMREAD_GRAYSCALE)
    image2 = cv2.imread('.\\..\\Datasets\\000001.png', cv2.IMREAD_GRAYSCALE)

    # Ensure the images are loaded properly
    if image1 is None or image2 is None:
        print("Error loading one of the images")
        return

    # Detect and compute keypoints and descriptors for both frames
    frame1.detectAndCompute(image1)
    frame2.detectAndCompute(image2)

    # Match keypoints between the two frames
    goodMatches = MatchKeyPoints(frame1, frame2)

    if goodMatches:
        # Draw matches
        matched_image = draw_matches_vertically(image1, frame1.KeyPoints(), image2, frame2.KeyPoints(), goodMatches)
        cv2.imshow('Matches', matched_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No good matches found.")


# This ensures that the main function only runs when this script is executed directly.
if __name__ == "__main__":
    main()