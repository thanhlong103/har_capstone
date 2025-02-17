import cv2
import numpy as np

# Function to detect and track the pupil
def track_pupil(image):
    # Convert to grayscale
    # Convert to grayscale.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and improve circle detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply thresholding to segment the pupil region
    threshold = cv2.inRange(gray, 0,50)

    # Perform Edge Detection.
    low_threshold = 50
    high_threshold = 100
    canny_blur = cv2.Canny(threshold, low_threshold, high_threshold)

    # Use Hough Circle Transform to detect circles (pupil)
    circles = cv2.HoughCircles(canny_blur, cv2.HOUGH_GRADIENT, dp=1, minDist=0.2,
                               param1=100, param2=30)
    

    # If circles are detected, process them
    if circles is not None:
        circles = np.uint16(np.around(circles))
        circle_x = []
        circle_y = []
        radius_list = []
        for circle in circles[0, :]:
            circle_x.append(circle[0])
            circle_y.append(circle[1])
            radius_list.append(circle[2])

        center = (int(np.mean(circle_x)), int(np.mean(circle_y))) # center of the circle
        print(center)
        radius = int(np.mean(radius_list)*1.5)  # radius of the circle
        # Draw the circle in the output image
        cv2.circle(image, center, radius, (0, 255, 0), 2)  # green circle for pupil
        # Draw the center of the circle
        cv2.circle(image, center, 2, (0, 0, 255), 3)  # red center for pupil

    return image

# Main loop for capturing video
cap = cv2.VideoCapture("480106035_9387289661338795_890617184540908566_n.mp4")  # Use the first webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Call the track_pupil function on each frame
    output_frame = track_pupil(frame)

    # Show the result in a window
    cv2.imshow('Pupil Tracker', output_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
