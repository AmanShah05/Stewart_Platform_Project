import cv2 as cv
import numpy as np

def detect_yellow_ball():
    # Start capturing video from the webcam
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    # Set the frame resolution
    width, height = 480, 480
    hex_center = (width // 2, height // 2)  # Center of the frame

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Resize the frame
        frame = cv.resize(frame, (width, height))

        # Create a hexagonal mask
        hex_mask = np.zeros((height, width), dtype=np.uint8)
        hex_radius = min(width, height) // 2

        # Define the points of the hexagon
        hex_points = np.array([
            [int(hex_center[0] + hex_radius * np.cos(np.radians(angle))),
             int(hex_center[1] + hex_radius * np.sin(np.radians(angle)))]
            for angle in range(0, 360, 60)
        ])
        # Fill the hexagon on the mask
        cv.fillConvexPoly(hex_mask, hex_points, 255)

        # Convert the frame to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define the range of yellow color in HSV
        ball_color_lower = np.array([20, 100, 100])
        ball_color_upper = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        color_mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)

        # Apply the hexagonal mask to the color mask to filter yellow only in hexagon
        masked_color = cv.bitwise_and(color_mask, hex_mask)

        # Find contours in the masked color
        contours, _ = cv.findContours(masked_color, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10:
                # Calculate the coordinates of the ball relative to the center
                x_rel = int(x - hex_center[0])
                y_rel = int(y - hex_center[1])

                # Draw a yellow circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                
                yield int(x_rel), int(y_rel)
                # Display the relative position of the ball
                # print(f"Yellow ball detected at relative position: ({x_rel}, {y_rel})")

        # Apply the hexagonal mask to the frame for display
        hex_frame = cv.bitwise_and(frame, frame, mask=hex_mask)

        # Display the resulting frame with hexagon mask applied
        cv.imshow('Hexagon Frame', hex_frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv.destroyAllWindows()

# Call the function to detect the yellow ball
for x, y in detect_yellow_ball():
    print("Detected yellow ball position:", (x, y))

