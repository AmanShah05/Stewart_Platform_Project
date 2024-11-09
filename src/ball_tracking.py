import cv2 as cv
import numpy as np

def detect_yellow_ball():
    # Initialize webcam feed
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)

    # Set the frame resolution to 480x480
    frame_width = 480
    frame_height = 480
    origin_x, origin_y = frame_width // 2, frame_height // 2  # Origin at center
    cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        
        # Resize frame and convert to HSV color space
        frame = cv.resize(frame, (frame_width, frame_height))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define the range for yellow color in HSV
        ball_color_lower = np.array([20, 100, 100])
        ball_color_upper = np.array([30, 255, 255])

        # Threshold the image to get only yellow colors
        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour and get its center and radius
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)

            if radius > 10:
                # Draw a circle around the detected ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)

                # Calculate the position relative to the center of the frame
                relative_x = int(x) - origin_x
                relative_y = int(y) - origin_y

                # Print and return the position
                print(f"Relative Ball Position to Center: (X: {relative_x}, Y: {relative_y})")

                # Use yield to return coordinates for each frame to calculate normal vector
                yield (relative_x, relative_y)

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop on 'q' key press
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

# Example usage of the function
for position in detect_yellow_ball():
    x, y = position
    # Here, you can further process x, y (e.g., to calculate the normal vector)
