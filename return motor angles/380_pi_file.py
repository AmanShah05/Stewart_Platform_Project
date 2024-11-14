import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import inverseKinPython
import normal_vector

# Given constants for PD control
K_p = 0.001425  # Proportional gain
K_d = 0.0001    # Derivative gain
PI = 3.1415926535
A, B, C = 0, 1, 2
# Initial error values (starting from 0 for the first call)

def plot_perpendicular_plane(ax, x, y, z, nx, ny, nz, size=100):
    # Create grid in the local x-y plane of the normal vector
    grid_range = np.linspace(-size, size, 10)
    xx, yy = np.meshgrid(grid_range, grid_range)


    # Equation of the plane: nx*(X - x) + ny*(Y - y) + nz*(Z - z) = 0
    if nz != 0:
        zz = (-nx * (xx - x) - ny * (yy - y)) / nz + z
    else:
        # Handle the case where nz is zero (plane is vertical)
        zz = np.full_like(xx, z)  # Make zz constant

    ax.scatter(x, y, 75, color='red', s=50)  # s sets marker size

    ax.quiver(0, 0, 0, nx, ny, nz,color='blue', length=150, normalize=True)
    # Plot the plane
    ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.3, edgecolor='blue')

def main():
    hz = 11
  
    # get error () from camera
    # test values
    x_curr = 250 
    y_curr = 250
    x_prev = 240
    y_prev = 240

    vec = normal_vector.calculate_normal_vector_PD(x_curr, y_curr, x_prev, y_prev)
    nx = vec[0]
    ny = vec[1]
    nz = vec[2]
    vec = [-nx,-ny, nz] # want opposite tilt

    pos = [0, 0, 0]  # Define pos as a list with three elements
    #vec = inverseKinPython.unitVec(0.1, 0.2)

    testA = inverseKinPython.platformAngle(A, vec) * 180 / PI
    testB = inverseKinPython.platformAngle(B, vec) * 180 / PI
    testC = inverseKinPython.platformAngle(C, vec) * 180 / PI

    print(f"Platform Angles: {testA}, {testB}, {testC}")

    motor_angle1 = inverseKinPython.motorAngle(inverseKinPython.platformAngle(A, vec), hz) * 180 / PI
    motor_angle2 = inverseKinPython.motorAngle(inverseKinPython.platformAngle(B, vec), hz) * 180 / PI
    motor_angle3 = inverseKinPython.motorAngle(inverseKinPython.platformAngle(C, vec), hz) * 180 / PI

    print(f"Motor Angles: {motor_angle1}, {motor_angle2}, {motor_angle2}")
    
    # Set up the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-250, 250])
    ax.set_ylim([-250, 250])
    ax.set_zlim([-100, 400])
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title("Normal Vector and Perpendicular Plane")
    plot_perpendicular_plane(ax,x_curr,y_curr,0,vec[0],vec[1],vec[2],100)

    plt.show()
    return motor_angle1, motor_angle2, motor_angle3
    

if __name__ == "__main__":
    main()
