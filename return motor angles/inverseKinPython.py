import math

# Calculation Variables
nmag, nz, nx_b, nx_c = 0, 0, 0, 0  # magnitude and z component of the normal vector
x, y, z = 0, 0, 0  # generic variables for the components of legs A, B, and C
mag = 0  # generic magnitude of the leg vector
angle = 0  # generic angle for legs A, B, and C
PI = math.pi
phi_a, phi_b, phi_c = 0, 0, 0

# Constants
A, B, C = 0, 1, 2

# All dimensions are in cm
R = 12.5  # platform radius
RM = 12.5  # radial distance from base centre to motors
L1 = 6.719  # bottom link length
L2 = 9.4592  # top link length

def unitVec(propX, propY):
    magn = math.sqrt(propX**2 + propY**2 + 1)  # magnitude

    vec = [propX / magn, propY / magn, 1 / magn]
    # check = vec[0]**2 + vec[1]**2 + vec[2]**2
    # print(f"Check unit vector magnitude: {check}")

    return vec

def platformAngle(leg, vec):
    # Take normal vector components
    nx, ny, nz = vec[0], vec[1], vec[2]

    # Determine normal radial component based on chosen leg
    if leg == A:
        nr = nx
    elif leg == B:
        nr = math.sqrt(0.75) * ny - 0.5 * nx
    elif leg == C:
        nr = -math.sqrt(0.75) * ny - 0.5 * nx

    # Return platform angle of incline for chosen leg, in radians
    return math.atan(-nr / nz)

def motorAngle(inclineAngle, height):
    # Determine position of sphere joint in (r, z) coordinates
    platform_r = R * math.cos(inclineAngle)
    platform_z = R * math.sin(inclineAngle) + height

    # Perform some intermediate calculations for reused variables
    dSq = (platform_r - RM)**2 + platform_z**2
    D = 1 + (L1**2 - L2**2) / dSq
    E = math.sqrt(L1**2 / dSq - 0.25 * D**2)

    # Determine position of pin joint in (r, z) coordinates
    joint_r = RM + 0.5 * D * (platform_r - RM) + platform_z * E
    joint_z = 0.5 * D * platform_z + (RM - platform_r) * E

    # Determine angle of motor based on pin joint position, in radians
    return math.atan(joint_z / (joint_r - RM))

def main():
    hz = 11
    nx = 0.342
    ny = 0.342

    pos = [0, 0, 0]  # Define pos as a list with three elements
    vec = unitVec(0.1, 0.2)

    testA = platformAngle(A, vec) * 180 / PI
    testB = platformAngle(B, vec) * 180 / PI
    testC = platformAngle(C, vec) * 180 / PI

    print(f"Platform Angles: {testA}, {testB}, {testC}")

    testA2 = motorAngle(platformAngle(A, vec), hz) * 180 / PI
    testB2 = motorAngle(platformAngle(B, vec), hz) * 180 / PI
    testC2 = motorAngle(platformAngle(C, vec), hz) * 180 / PI

    print(f"Motor Angles: {testA2}, {testB2}, {testC2}")

if __name__ == "__main__":
    main()
