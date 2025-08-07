import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

####################################################################################
# Note: This is where you implement the model order reduction using SVD.
####################################################################################
def reduction(data, mode=0, order=7):
    """
    Perform model order reduction on a matrix of node positions.

    Parameters:
    - data: np.ndarray, shape (n, L), full position data over time
    - mode: 0 to plot the reduction error, 1 to compute the reduction
    - order: number of modes to keep if mode == 1
    """

    # === Step 1: Apply SVD ===
    # M = U Σ V^T
    # U: spatial modes, Σ: singular values
    # TODO: Complete this part
    U, S, Vt = None, None, None  # <<< To be completed

    if mode == 0:
        # === Step 2: Calculate reduction error for different orders ===
        # The error is defined as: 1 - (sum of first r σ_i) / (total sum of σ_i)
        # TODO: Complete this part
        max_r = min(20, len(S))
        errors = []  # <<< To be completed

        # Affichage du graphe
        plt.figure(figsize=(6, 4))
        plt.bar(range(1, max_r + 1), errors)
        plt.xlabel("Ordre de réduction r")
        plt.ylabel("Erreur de réduction")
        plt.title("Erreur vs. nombre de modes conservés")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    elif mode == 1:
        if order > len(S):
            raise ValueError(f"Order too high: {order}. Maximum allowed is {len(S)}.")
        # === Step 3: Build the reduction matrix T from the first r columns of U ===
        # T will project the full position vector onto a lower-dimensional subspace.
        # TODO: Complete this part
        T = None  # <<< To be completed

        # === Step 4: Extend the reduction to both position AND velocity ===
        # The full state vector includes both position and velocity.
        # We build the full reduction matrix R as a block-diagonal matrix:
        #     R = [T 0; 0 T]
        # This ensures that both position and velocity are reduced in the same way.
        # TODO: Complete this part
        R = None  # <<< To be completed


        save_path = os.path.join(reduction_path, f"order{order}.npz")
        np.savez(save_path, reductionMatrix=R, reductionMatrixPos=T)

    else:
        raise ValueError("Mode inconnu. Utilisez 0 pour afficher l'erreur ou 1 pour calculer la réduction.")

####################################################################################
# Note: The following code is for setting up the environment and running the reduction.
####################################################################################
lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")
reduction_path = os.path.join(data_path, "reduction")
if not os.path.exists(reduction_path):
    os.makedirs(reduction_path, exist_ok=True)

# Argument parser
def get_parser_args():
    parser = argparse.ArgumentParser(description='Model order reduction using SVD.')
    parser.add_argument('--mode', type=int, default=0,
        help="0: plot error vs. order, 1: compute reduction")
    parser.add_argument('--order', type=int, default=7,
        help="Reduction order (used only if mode=1)")
    try:
        args = parser.parse_args()
    except SystemExit:
        # Allow running from interactive environments
        args = parser.parse_args([])
    return args

# Main reduction logic
def perform_reduction():
    args = get_parser_args()

    # Load data
    npz_path = os.path.join(data_path, "sofa", "openLoop.npz")
    if not os.path.exists(npz_path):
        raise FileNotFoundError(f"Data file not found at: {npz_path}. Please run the open-loop simulation first.")

    data = np.load(npz_path)
    positions = data["legsPos"].T  # Shape: (n, L) with n = nb of positions, L = nb of time steps
    reduction(positions, mode=int(args.mode), order=int(args.order))



# Optional SOFA interface compatibility
def createScene(rootnode):
    perform_reduction()

# Script entry point
if __name__ == "__main__":
    perform_reduction()
