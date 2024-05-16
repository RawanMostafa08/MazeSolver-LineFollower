import cv2
import numpy as np

def get_color_mask(image, color_lower1, color_upper1, color_lower2, color_upper2):
    # Convert image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for the color ranges
    lower_bound1 = np.array(color_lower1, dtype=np.uint8)
    upper_bound1 = np.array(color_upper1, dtype=np.uint8)
    lower_bound2 = np.array(color_lower2, dtype=np.uint8)
    upper_bound2 = np.array(color_upper2, dtype=np.uint8)
    
    # Create masks using the defined color ranges
    mask1 = cv2.inRange(hsv_image, lower_bound1, upper_bound1)
    mask2 = cv2.inRange(hsv_image, lower_bound2, upper_bound2)
    
    # Combine the masks using bitwise OR
    mask = cv2.bitwise_or(mask1, mask2)
    
    return mask

def get_sv_color(image, mask):
    # Convert image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Apply mask to the HSV image
    masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
    
    # Check if mask has any non-zero pixels
    if np.any(mask):
        # Calculate average S and V values within the mask
        s_values = masked_image[:,:,1][mask > 0]
        v_values = masked_image[:,:,2][mask > 0]
        
        avg_s = np.mean(s_values)
        avg_v = np.mean(v_values)
    else:
        # If mask is empty, return None for averages
        avg_s = None
        avg_v = None
    
    return avg_s, avg_v

# Read the input image
image = cv2.imread('yellow.jpeg')

# Define the suitable color range for yellow (in HSV format)
# Yellow color has a wide range due to its hue
yellow_lower1 = [20, 100, 100]    # Lower bound (Hue, Saturation, Value)
yellow_upper1 = [30, 255, 255]    # Upper bound (Hue, Saturation, Value)
yellow_lower2 = [25, 100, 100]    # Lower bound (Hue, Saturation, Value) - another range for yellow
yellow_upper2 = [35, 255, 255]    # Upper bound (Hue, Saturation, Value) - another range for yellow

# Get the mask for the suitable color ranges
mask = get_color_mask(image, yellow_lower1, yellow_upper1, yellow_lower2, yellow_upper2)

# Get average S and V values for the suitable color
avg_s, avg_v = get_sv_color(image, mask)

if avg_s is not None and avg_v is not None:
    print("Average S value:", avg_s)
    print("Average V value:", avg_v)
else:
    print("No pixels found in the specified color range.")

# Display the mask
cv2.imshow('Mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
