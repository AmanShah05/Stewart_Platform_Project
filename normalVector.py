import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

# Given constants and parameters
x_max = 480
y_max = 480

x_centre = x_max / 2
y_centre = y_max / 2

x_curr = 480
y_curr = 240

nx_max = 0.342
ny_max = 0.342

#K_p = nx_max/(x_max/2)
K_p = 0.001425
K_d = 0.0001 



# Function to calculate the normal vector based on the proportional control
def calculate_normal_vector_PD(error_x, error_y, d_x, d_y):
    # Proportional control outputs
    
    nx = K_p * error_x + K_d * d_x
    ny = K_p * error_y + K_d * d_y

    # Calculate nz to ensure the normal vector remains a unit vector
    nz = math.sqrt(1 - nx**2 - ny**2)
    print(f"nx: {nx} \n ny:{ny} \n nz:{nz}")

    return nx, ny, nz

def calculate_normal_vector_P(error_x, error_y):
    # Proportional control outputs
    nx = K_p * error_x
    ny = K_p * error_y

    # Calculate nz to ensure the normal vector remains a unit vector
    nz = math.sqrt(1 - nx**2 - ny**2)
    print(f"nx: {nx} \n ny:{ny} \n nz:{nz}")

    return nx, ny, nz

def plot_perpendicular_plane(ax, x, y, z, nx, ny, nz, size=100):
    # Create grid in the local x-y plane of the normal vector
    grid_range = np.linspace(-size, size, 10)
    xx, yy = np.meshgrid(grid_range, grid_range)
    
    xx += x
    yy += y

    # Equation of the plane: nx*(X - x) + ny*(Y - y) + nz*(Z - z) = 0
    if nz != 0:
        zz = (-nx * (xx - x) - ny * (yy - y)) / nz + z
    else:
        # Handle the case where nz is zero (plane is vertical)
        zz = np.full_like(xx, z)  # Make zz constant
    
    # Plot the plane
    ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.3, edgecolor='blue')


# Combine x and y into an array of coordinate pairs
x_error = x_curr - x_centre
y_error = y_curr - y_centre

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0, 500])
ax.set_ylim([0, 500])
ax.set_zlim([-100, 400])
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("Normal Vector and Perpendicular Plane")

def test_movement(x_coords, y_coords):
    num_coords = len(x_coords)
    x_prev = 0
    y_prev = 0
    for i in range(num_coords):
        ax.cla()
        deltaT = 0.05

        ax.set_xlim([0, 500])
        ax.set_ylim([0, 500])
        ax.set_zlim([-100, 400])
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")
        ax.set_zlabel("Z Axis")

        x_curr = x_coords[i]
        y_curr = y_coords[i]

        x_error = x_curr - x_centre
        y_error = y_curr - y_centre

        x_derivative = (x_error - x_prev)
        y_derivative = (y_error - y_prev)

        # nx, ny, nz = calculate_normal_vector_P(x_error, y_error)
        nx, ny, nz = calculate_normal_vector_PD(x_error, y_error, x_derivative, y_derivative)


        ax.scatter(x_curr, y_curr, 0, color='red', s=50)  # s sets marker size

        ax.quiver(240, 240, 0, -nx, -ny, nz,color='blue', length=150, normalize=True)

        plot_perpendicular_plane(ax, 240, 240, 0, -nx, -ny, nz, size=200)

        x_prev = x_error
        y_prev = y_error

        plt.pause(0.05)

# Generate x coordinates from 0 to 480 with a step of 5
x_coords1 = np.arange(0, 481, 5)
y_coords1 = np.arange(0, 481, 5)

x_coords_origin = np.full(x_coords1.shape, 240)
y_coords_origin = np.full(y_coords1.shape, 240)

test_movement(x_coords1,y_coords_origin)
test_movement(x_coords_origin,y_coords1)
test_movement(x_coords1,y_coords1)
plt.show()




