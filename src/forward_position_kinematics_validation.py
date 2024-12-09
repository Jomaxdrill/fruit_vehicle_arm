import math
import sympy as sp
import numpy as np
DECIMALS = 10  # used for rounding values in matrix, when computing an homogeneous matrix with actual number.
theta1, theta2, theta3, theta4 = sp.symbols("theta1, theta2, theta3, theta4")  # symbols for joint angles

# Denavit-Hartenberg Table for UR3 robot
# theta, alpha, a, d = link (note that some measurements x 0.254m/in to convert from inch to meter)
# dh_table = ((theta1, -sp.pi/2, 0, (4.5+4.65)*0.0254),
#             (theta2+sp.pi/2, sp.pi, -6*0.0254, 0),
#             (theta3+sp.pi/2, 0, 6*0.0254, 0),
#             (theta4, 0, 2*0.0254, 0))

# dh_table in inches
dh_table = ((theta1, -sp.pi/2, 0, 4.5+4.65),
            (theta2+sp.pi/2, sp.pi, -6, 0),
            (theta3+sp.pi/2, 0, 6, 0),
            (theta4, 0, 2, 0))

def clean_zeros(matrix):
    """
    Converts negative zeros to positive zeros in a numpy array.
    
    Parameters
    ----------
    matrix : numpy.ndarray
        Input matrix that may contain negative zeros
        
    Returns
    -------
    numpy.ndarray
        Matrix with negative zeros converted to positive zeros
    """
    # np.where(condition, value_if_true, value_if_false). takes absolute values of entries & set those < 10^-10 to 0.
    return np.where(np.abs(matrix) < 1e-10, 0.0, matrix)

def ask_for_validation() -> bool:
    """
    Asks the user if they want to perform validation.
    Returns
    -------
    bool
        True if validation is requested, False otherwise.
    """
    while True:
        try:
            choice = int(input("Would you like to validate the end-effector transform with some joint angle values? Enter 1 for yes or 0 for no: "))
            if choice in (0, 1):
                return bool(choice)
            else:
                print("Please enter 1 for yes or 0 for no.")
        except:
            print("Invalid input. Please enter 1 or 0.")

def get_joint_angles() -> tuple:
    """
    Prompts the user to enter joint angles.
    Returns
    -------
    tuple
        A tuple of six joint angles in radians.
    """
    while True:
        try:
            angles = input("Enter 4 joint angles in degrees, separated by commas (no commas after last angle): ")
            angles = [float(value.strip()) for value in angles.split(",")]
            if len(angles) != 4:
                print("Please enter exactly six angles.")
                continue
            print(angles)
            return tuple(math.radians(angle) for angle in angles)
        except:
            print("Invalid input. Please enter numeric values separated by commas.")

def calc_ht_matrix(link: tuple) -> sp.Matrix:
    """
    Calculates the homogeneous transformation matrix for a given link.
    Parameters
    ----------
    link : tuple
        Link parameters (theta, alpha, a, d) from the DH table.
    Returns
    -------
    sympy.Matrix
        The homogeneous transformation matrix as a SymPy matrix.
    """
    theta, alpha, a, d = link
    
    homogeneous_transform = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
    [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
    [0, sp.sin(alpha), sp.cos(alpha), d],
    [0, 0, 0, 1]])
    return homogeneous_transform

def calc_end_effector_transform() -> sp.Matrix:
    """
    Computes the symbolic end-effector homogeneous transformation matrix.
    Returns
    -------
    sympy.Matrix
        The symbolic end-effector transformation matrix.
    """
    transM_1_0 = calc_ht_matrix(dh_table[0])
    transM_2_1 = calc_ht_matrix(dh_table[1])
    transM_3_2 = calc_ht_matrix(dh_table[2])
    transM_4_3 = calc_ht_matrix(dh_table[3])
    transM_4_0 = transM_1_0 * transM_2_1 * transM_3_2 * transM_4_3
    return transM_4_0

def main():
    """ Main function to run the code"""
    transM_4_0 = calc_end_effector_transform()
    print("the homogeneous transform for the end-effector w.r.t the base frame is: \n")
    sp.pprint(transM_4_0)
    print("-"*200, '\n')
    if ask_for_validation():  # ask user if they want to do forward position kinematics validation, if true, then continue
        tht1, tht2, tht3, tht4 = get_joint_angles()  # ask user to enter joint angles
        # Compute the matrix and clean the negative zeros
        transM_4_0_clc = clean_zeros(
            np.round(
                sp.matrix2numpy(
                    transM_4_0.subs({
                        theta1: tht1, 
                        theta2: tht2, 
                        theta3: tht3, 
                        theta4: tht4
                    }), 
                    dtype="float"
                ), 
                decimals=DECIMALS
            )
        )
        # finding x,y,z end-effector coordinates from the last column of the transformation matrix
        x_e, y_e, z_e = transM_4_0_clc[:3, 3]
        print("\nend-effector transformation matrix computed with the chosen joint angles is: ")
        print("\n", transM_4_0_clc)
        print(f"\nthe (x, y, z) coordinate of the end-effector is: {(x_e, y_e, z_e)} inches")
        inch_to_meter = 0.0254
        print(f"\nthe (x, y, z) coordinate of the end-effector is: {round(x_e*inch_to_meter,4), round(y_e*inch_to_meter,4),round(z_e*inch_to_meter,4)} meters")

if __name__ == "__main__":
    main()