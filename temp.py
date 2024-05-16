import cv2
import numpy as np

def detect_lines(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply thresholding to detect black lines
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on length to identify potential lines
    potential_lines = [cnt for cnt in contours if cv2.arcLength(cnt, False) > 100]

    # Draw lines on the original image
    line_image = np.zeros_like(image)
    cv2.drawContours(line_image, potential_lines, -1, (255, 255, 255), 2)

    return line_image

def get_robot(image):
    # Convert the image to HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the HSV range for red
    mask1 = cv2.inRange(img_hsv, (0, 50, 20), (5, 255, 255))
    mask2 = cv2.inRange(img_hsv, (175, 50, 20), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours in the red mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Assuming one contour is found
    (x_axis, y_axis), radius = cv2.minEnclosingCircle(max(contours, key=len))
    center = (int(x_axis), int(y_axis))
    radius = int(radius)

    cv2.circle(image, center, radius, (0, 255, 0), 2)

    # Display the original image with the minimum enclosing circle
    cv2.imshow('Image with Minimum Enclosing Circle', image)

    return center, image

def determine_distance_to_line(center, line_center, straight_line_threshold=100):
    distance = abs(center[0] - line_center[0])
    if(distance < straight_line_threshold):
        print("strigth line 1")
    else:
        print("curved line 0")
    return distance

# Specify the correct path to the image
image_path = "f2.png"
image = cv2.imread(image_path)
dim = (800, 800)
frame = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

# Process the image
line_image = detect_lines(frame)
line_center = (frame.shape[1] // 2, frame.shape[0] // 2)  # Center of the image width

# Display the image with detected lines
cv2.imshow('Image with Detected Lines', line_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Process the image to get the center of the red object
red_center, processed_image = get_robot(frame)

# Determine the distance between the red object and the detected line
distance_to_line = determine_distance_to_line(red_center, line_center)
print(f"Distance between the red object and the detected line: {distance_to_line} pixels")

# Display the processed image if available
cv2.imshow('Processed Image', processed_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
