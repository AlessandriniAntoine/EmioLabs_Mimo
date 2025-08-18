import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

####################################################################################
# Note: This is where you implement the identification of the reduced linear model.
####################################################################################
def identify_model(states, inputs, outputs, reduced_positions, r):
    """
    Identify the discrete-time state-space model from reduced data.

    Parameters:
    - states: np.ndarray, shape (nx, L), reduced full states over time
    - inputs: np.ndarray, shape (nu, L), inputs over time
    - outputs: np.ndarray, shape (ny, L), outputs over time
    - reduced_positions: np.ndarray, shape (nr, L), reduced positions over time
    - r: int, reduction order (number of modes kept)

    Returns:
    - A: State matrix
    - B: Input matrix
    - C: Output matrix
    """

    # === Step 1: Split current and next states ===
    x_next = states[:, 1:]
    x_curr = states[:, :-1]
    u_curr = inputs[:, :-1]

    # === Step 2: Compute state and input matrices using least-squares ===
    # Stack state and input to form [x; u]
    # TODO: Complete this part
    state_input = np.vstack((x_curr, u_curr))  # Shape: (nx + nu, L - 1)
    AB = x_next @ np.linalg.pinv(state_input)  # Shape: (nx, nx + nu)

    # Extract A and B from AB
    A = AB[:, :states.shape[0]]  # State matrix A
    B = AB[:, states.shape[0]:]  # Input matrix B

    # === Step 3: Compute output matrix C ===
    # C = [0  Cp] where Cp maps reduced positions to outputs
    # TODO: Complete this part
    Cp = outputs @ np.linalg.pinv(reduced_positions)  # Shape: (ny, np)
    C = np.hstack((np.zeros_like(Cp), Cp))  # Shape: (ny, nx)

    return A, B, C

####################################################################################
# Note: The following code is provided and loads data, handles paths, and runs logic.
####################################################################################

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")


def get_parser_args():
    parser = argparse.ArgumentParser(description='Identify a reduced state-space model.')
    parser.add_argument('--order', type=int, default=6,
        help="Reduction order used to build the model")
    try:
        args = parser.parse_args()
    except SystemExit:
        args = parser.parse_args([])
    return args

def perform_identification():
    args = get_parser_args()

    # Load recorded data
    openloop_path = os.path.join(data_path, "sofa_openLoop.npz")
    if not os.path.exists(openloop_path):
        raise FileNotFoundError(f"Data file not found at: {openloop_path}. Please run the open-loop simulation first.")
    data = np.load(openloop_path)
    legs_pos = data["legsPos"].T
    legs_vel = data["legsVel"].T
    states_full = np.vstack((legs_vel, legs_pos))  # Full state (v; p)
    outputs = data["markersPos"].T
    inputs = data["motorsPos"].T

    # Load reduction matrix
    reduction_file = os.path.join(data_path, f"reduction_order{args.order}.npz")
    if not os.path.exists(reduction_file):
        raise FileNotFoundError(f"Reduction file not found at: {reduction_file}. Please run the reduction script first.")
    red = np.load(reduction_file)
    T = red["reductionMatrixPos"]
    R = red["reductionMatrix"]

    # Reduce states and positions
    reduced_states = R.T @ states_full
    reduced_positions = T.T @ legs_pos

    # Identify the model
    A, B, C = identify_model(reduced_states, inputs, outputs, reduced_positions, args.order)

    # Print dimensions
    print(f"System matrices:\nA: {A.shape}, B: {B.shape}, C: {C.shape}")

    # === Optional: Predict and simulate model ===
    L = reduced_states.shape[1]
    nx = A.shape[0]

    # Prediction using known data
    x_pred = A @ reduced_states + B @ inputs
    y_pred = C @ x_pred

    # Simulation from initial condition
    x_sim = reduced_states[:, 0].reshape(-1, 1)
    X_sim = np.zeros_like(reduced_states)
    Y_sim = np.zeros_like(outputs)
    X_sim[:, 0] = x_sim[:, 0]
    Y_sim[:, 0] = (C @ x_sim)[:, 0]
    for k in range(1, L):
        u = inputs[:, k-1].reshape(-1, 1)
        x_sim = A @ x_sim + B @ u
        X_sim[:, k] = x_sim[:, 0]
        Y_sim[:, k] = (C @ x_sim)[:, 0]

    # Plot results
    fig, axes = plt.subplots(nx, 1, figsize=(6, 2*nx))
    for i in range(nx):
        axes[i].plot(reduced_states[i], label="Original", color="red")
        axes[i].plot(x_pred[i], label="Prediction", linestyle="--", color="blue")
        axes[i].plot(X_sim[i], label="Simulation", linestyle="--", color="green")
        axes[i].set_ylabel(f"$x_{i}$")
    axes[0].legend()
    fig.suptitle("Reduced States")

    ny = outputs.shape[0]
    fig, axes = plt.subplots(ny, 1, figsize=(6, 2*ny))
    for i in range(ny):
        axes[i].plot(outputs[i], label="Original", color="red")
        axes[i].plot(y_pred[i], label="Prediction", linestyle="--", color="blue")
        axes[i].plot(Y_sim[i], label="Simulation", linestyle="--", color="green")
        axes[i].set_ylabel(f"$y_{i}$")
    axes[0].legend()
    fig.suptitle("Outputs")
    plt.show()

    # Save model
    np.savez(os.path.join(data_path, f"model_order{args.order}.npz"),
             stateMatrix=A,
             inputMatrix=B,
             outputMatrix=C)

# Optional SOFA interface
def createScene(root):
    perform_identification()

# Entry point
if __name__ == "__main__":
    perform_identification()
