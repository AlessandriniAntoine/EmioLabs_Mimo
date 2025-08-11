import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import control as ct

####################################################################################
# Note: This is where you design the observer gain matrix L.
####################################################################################
def design_observer(A, C):
    """
    Design an observer gain L such that the estimation error converges.

    Parameters:
    - A: np.ndarray, shape (nx, nx), state matrix
    - C: np.ndarray, shape (ny, nx), output matrix

    Returns:
    - L: np.ndarray, shape (nx, ny), observer gain matrix
    """

    # === Step 0: Check the observability of the system ===
    obsv_rank = np.linalg.matrix_rank(ct.obsv(A, C))
    if obsv_rank != A.shape[0]:
        raise ValueError(f"System is not observable: rank {obsv_rank}, expected {A.shape[0]}.")

    # === Step 1: Solve the dual control problem to get L ===
    # Observer design is dual: use A.T, C.T, Q, R
    # TODO: Compute observer gain L
    Q = 1e2 * np.eye(A.shape[0])
    R = 1e1 * np.eye(C.shape[0])
    Lt, _, _ =  ct.dlqr(A.T, C.T, Q, R)
    L = Lt.T

    # === Step 4: Compare eigenvalues ===
    # TODO: Compute the eigenvalues of the open-loop and observer systems
    eig_open = np.linalg.eigvals(A)
    print(f"Open-loop eigenvalue magnitudes: {np.abs(eig_open)}")
    # Display observer eigenvalues
    eig_obs = np.linalg.eigvals(A - L @ C)
    print(f"Observer eigenvalue magnitudes: {np.abs(eig_obs)}")
    return L

####################################################################################
# Note: The following code loads data, handles paths, and runs the observer logic.
####################################################################################

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")
model_path = os.path.join(data_path, "models")
control_path = os.path.join(data_path, "control")
if not os.path.exists(control_path):
    os.makedirs(control_path, exist_ok=True)

def get_parser_args():
    parser = argparse.ArgumentParser(description='Design an observer.')
    parser.add_argument('--order', type=int, default=6,
        help="Reduction order used for the model")
    try:
        args = parser.parse_args()
    except SystemExit:
        args = parser.parse_args([])
    return args

def perform_observer_design():
    args = get_parser_args()

    # Load identified model
    model_file = os.path.join(model_path, f"order{args.order}.npz")
    if not os.path.exists(model_file):
        raise FileNotFoundError(f"Model file not found: {model_file}. Please run identification first.")
    model = np.load(model_file)
    A = model["stateMatrix"]
    C = model["outputMatrix"]

    # Sanity check: observability
    obsv_rank = np.linalg.matrix_rank(ct.obsv(A, C))
    if obsv_rank != A.shape[0]:
        raise ValueError(f"System is not observable: rank {obsv_rank}, expected {A.shape[0]}.")

    # Design observer
    L = design_observer(A, C)

    # Save observer
    np.savez(os.path.join(control_path, f"order{args.order}_obs.npz"),
             observerGain=L)

# Optional SOFA interface
def createScene(root):
    perform_observer_design()

# Entry point
if __name__ == "__main__":
    perform_observer_design()
