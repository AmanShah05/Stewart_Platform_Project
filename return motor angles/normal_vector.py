import math

# Given constants for PD control
K_p = 0.001425  # Proportional gain
K_d = 0.0010   # Derivative gain
# Initial error values (starting from 0 for the first call)



# Function to calculate the normal vector based on PD control
def calculate_normal_vector_PD(x_curr, y_curr,x_prev,y_prev):
    # Calculate the error values (current error - previous error)
    error_x = x_curr  # Error for x: difference between current and previous x
    error_y = y_curr  # Error for y: difference between current and previous y

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
