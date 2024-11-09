import math

# Given constants for PD control
K_p = 0.001425  # Proportional gain
K_d = 0.0001    # Derivative gain
# Initial error values (starting from 0 for the first call)
x_prev = 0
y_prev = 0


# Function to calculate the normal vector based on PD control
def calculate_normal_vector_PD(x_curr, y_curr):
    # Calculate the error values (current error - previous error)
    error_x = x_curr - x_prev  # Error for x: difference between current and previous x
    error_y = y_curr - y_prev  # Error for y: difference between current and previous y

    # Derivatives of the error (change in error)
    x_derivative = error_x - x_prev  # Rate of change of error for x
    y_derivative = error_y - y_prev  # Rate of change of error for y

    # Apply PD control to calculate the normal vector components
    nx = K_p * error_x + K_d * x_derivative
    ny = K_p * error_y + K_d * y_derivative

    # Calculate the magnitude of the normal vector
    nmag = math.sqrt(nx**2 + ny**2 + 1)  # Since nz = 1, we calculate the magnitude

    # Normalize the components of the normal vector
    nx /= nmag
    ny /= nmag
    nz = 1 / nmag  # Since nz is fixed to 1

    # Print the normalized normal vector
    print(f"nx: {nx} \nny: {ny} \nnz: {nz}")

    # Overwrite the previous error values with the current ones
    x_prev = error_x
    y_prev = error_y

    return nx, ny, nz  # Return the errors for the next iteration


# Example usage
x_curr = 0.5  # Current x value (example)
y_curr = 0.5  # Current y value (example)

# Call the function with initial values
nx, ny, nz= calculate_normal_vector_PD(x_curr, y_curr)
