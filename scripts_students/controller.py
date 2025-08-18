import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import control as ct

####################################################################################
# Note: This is where you design the state feedback controller (LQR) and compute G.
####################################################################################
def design_controller(A, B, C):
    """
    Compute the feedback gain K and feedforward gain G for reference tracking.

    Parameters:
    - A: np.ndarray, shape (nx, nx), state matrix
    - B: np.ndarray, shape (nx, nu), input matrix
    - C: np.ndarray, shape (ny, nx), output matrix

    Returns:
    - K: np.ndarray, shape (nu, nx), state feedback gain
    - G: np.ndarray, shape (nu, ny), feedforward gain for reference tracking
    """
    # === Step 0: Check the controllability of the system ===
    # TODO: Check if the system is controllable
    ctrb_rank = None # <<< To be completed
    if ctrb_rank != A.shape[0]:
        raise ValueError(f"System is not controllable: rank {ctrb_rank}, expected {A.shape[0]}.")

    # === Step 1: Select the outputs to control ===
    # Define the controlled output matrix C_ctr
    C_ctr = None # <<< To be completed

    if C_ctr.shape[0] != B.shape[1]:
        raise ValueError(f"Controlled output must have the same dimension as the input. Got {C_ctr.shape[0]} controlled outputs and {B.shape[1]} inputs.")

    # === Step 2: Compute the feedback gain K ===
    # TODO: Use one of the methods to compute the state feedback gain K
    K = None  # <<< To be completed

    # === Step 3: Compute the feedforward gain G ===
    # TODO: Compute G to track constant references on controlled outputs
    G_inv = None  # <<< To be completed
    G = None      # <<< To be completed

    # === Step 4: Check the closed-loop eigenvalues ===
    eig_open = None # << To be completed
    print(f"Open-loop eigenvalue magnitudes: {np.abs(eig_open)}")
    eig_closed = None # <<< To be completed
    print(f"Closed-loop eigenvalue magnitudes: {np.abs(eig_closed)}")
    return K, G

####################################################################################
# Note: The following code loads data, handles paths, and runs the controller logic.
####################################################################################

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")


def get_parser_args():
    parser = argparse.ArgumentParser(description='Design a state feedback controller.')
    parser.add_argument('--order', type=int, default=6,
        help="Reduction order used for the model")
    try:
        args = parser.parse_args()
    except SystemExit:
        args = parser.parse_args([])
    return args

def perform_controller_design():
    args = get_parser_args()

    # Load identified model
    model_file = os.path.join(data_path, f"model_order{args.order}.npz")
    if not os.path.exists(model_file):
        raise FileNotFoundError(f"Model file not found: {model_file}. Please run identification first.")
    model = np.load(model_file)
    A = model["stateMatrix"]
    B = model["inputMatrix"]
    C = model["outputMatrix"]

    # Sanity check: controllability
    ctrb_rank = np.linalg.matrix_rank(ct.ctrb(A, B))
    if ctrb_rank != A.shape[0]:
        raise ValueError(f"System is not controllable: rank {ctrb_rank}, expected {A.shape[0]}.")

    # Design controller
    K, G = design_controller(A, B, C)

    # Save controller
    np.savez(os.path.join(data_path, f"controller_order{args.order}.npz"),
             feedbackGain=K,
             feedForwardGain=G)

# Optional SOFA interface
def createScene(root):
    perform_controller_design()

# Entry point
if __name__ == "__main__":
    perform_controller_design()
