import joblib
import pickle
import numpy as np
import pandas as pd
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import matplotlib.pyplot as plt


class InverseTorqueController(nn.Module):
    def __init__(self, n_state_features):
        super().__init__()
        input_size = 1 + n_state_features
        self.net = nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Sigmoid(),
        )

    def forward(self, target_torque, state_features):
        x = torch.cat([target_torque, state_features], dim=1)
        return self.net(x)


class InverseTorqueDataset(Dataset):
    def __init__(self, target_torques, state_features, throttles):
        self.target_torques = torch.from_numpy(target_torques)
        self.state_features = torch.from_numpy(state_features)
        self.throttles = torch.from_numpy(throttles)

    def __len__(self):
        return len(self.target_torques)

    def __getitem__(self, idx):
        return (self.target_torques[idx], self.state_features[idx], self.throttles[idx])


def test_inverse_controller(
    test_data_path,
    model_path="tempres/best_inverse_torque_controller.pth",
    torque_scaler_path="tempres/torque_scaler.pkl",
    state_scaler_path="tempres/state_scaler.pkl",
):
    """Test the inverse torque controller on new data"""

    print("Loading scalers...")
    torque_scaler = joblib.load(torque_scaler_path)
    state_scaler = joblib.load(state_scaler_path)

    print(f"Loading test data from {test_data_path}...")
    with open(test_data_path, "rb") as f:
        test_data = pickle.load(f)

    state_dict = test_data[("ego", "gtstate")]
    df_test = pd.DataFrame(state_dict)

    # Same preprocessing as training
    state_keys = [
        "RPM",
        "gearRatio",
        "engineLoad",
        "wheelRR_speed",
        "wheelRL_speed",
        "brakeInput",
        "steeringInput",
    ]

    df_test["avg_rear_torque"] = (
        df_test["wheelRR_propTorque"] + df_test["wheelRL_propTorque"]
    ) / 2

    # Filter data (same as training)
    df_clean = df_test[
        (df_test["brakeInput"] < 0.1) & (df_test["avg_rear_torque"] > 0)
    ].copy()

    target_torques = df_clean[["avg_rear_torque"]].to_numpy(dtype=np.float32)
    state_features = df_clean[state_keys].to_numpy(dtype=np.float32)
    actual_throttles = df_clean[["throttleInput"]].to_numpy(dtype=np.float32)

    # Apply scaling
    target_torques_scaled = torque_scaler.transform(target_torques)
    state_features_scaled = state_scaler.transform(state_features)

    # Load model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = InverseTorqueController(len(state_keys)).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    # Create test dataset
    test_ds = InverseTorqueDataset(
        target_torques_scaled, state_features_scaled, actual_throttles
    )
    test_loader = DataLoader(test_ds, batch_size=256, shuffle=False)

    # Make predictions
    print("Making predictions...")
    predictions = []
    actuals = []

    with torch.no_grad():
        for target_batch, state_batch, throttle_batch in test_loader:
            target_batch = target_batch.to(device)
            state_batch = state_batch.to(device)

            pred_throttle = model(target_batch, state_batch)
            predictions.append(pred_throttle.cpu().numpy())
            actuals.append(throttle_batch.numpy())

    y_pred = np.concatenate(predictions, axis=0).flatten()
    y_true = np.concatenate(actuals, axis=0).flatten()

    # Calculate metrics
    mse = mean_squared_error(y_true, y_pred)
    mae = mean_absolute_error(y_true, y_pred)
    rmse = np.sqrt(mse)
    r2 = r2_score(y_true, y_pred)

    print("\n" + "=" * 60)
    print("INVERSE TORQUE CONTROLLER TEST RESULTS")
    print("=" * 60)
    print(f"Mean Squared Error (MSE): {mse:.6f}")
    print(f"Root Mean Squared Error (RMSE): {rmse:.6f}")
    print(f"Mean Absolute Error (MAE): {mae:.6f}")
    print(f"R² Score: {r2:.6f}")
    print(f"Number of test samples: {len(y_true)}")
    print(f"Throttle prediction range: {y_pred.min():.3f} to {y_pred.max():.3f}")
    print(f"Actual throttle range: {y_true.min():.3f} to {y_true.max():.3f}")

    # Plotting
    plt.figure(figsize=(15, 10))

    # Scatter plot
    plt.subplot(2, 3, 1)
    plt.scatter(y_true, y_pred, alpha=0.5, s=1)
    plt.plot([y_true.min(), y_true.max()], [y_true.min(), y_true.max()], "r--", lw=2)
    plt.xlabel("Actual Throttle")
    plt.ylabel("Predicted Throttle")
    plt.title(f"Predictions vs Actual\nR² = {r2:.4f}")
    plt.grid(True, alpha=0.3)

    # Error distribution
    plt.subplot(2, 3, 2)
    errors = y_pred - y_true
    plt.hist(errors, bins=50, alpha=0.7, edgecolor="black")
    plt.xlabel("Prediction Error")
    plt.ylabel("Frequency")
    plt.title(f"Error Distribution\nMean: {errors.mean():.4f}, Std: {errors.std():.4f}")
    plt.grid(True, alpha=0.3)

    # Time series (first 1000 samples)
    plt.subplot(2, 3, 3)
    n_samples = min(1000, len(y_true))
    indices = np.arange(n_samples)
    plt.plot(indices, y_true[:n_samples], label="Actual", alpha=0.7)
    plt.plot(indices, y_pred[:n_samples], label="Predicted", alpha=0.7)
    plt.xlabel("Sample Index")
    plt.ylabel("Throttle")
    plt.title(f"Time Series (First {n_samples} samples)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Torque vs throttle relationship
    plt.subplot(2, 3, 4)
    torque_unscaled = target_torques.flatten()
    plt.scatter(torque_unscaled, y_true, alpha=0.5, s=1, label="Actual", color="blue")
    plt.scatter(torque_unscaled, y_pred, alpha=0.5, s=1, label="Predicted", color="red")
    plt.xlabel("Target Torque")
    plt.ylabel("Throttle")
    plt.title("Torque vs Throttle Relationship")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Performance by torque range
    plt.subplot(2, 3, 5)
    n_bins = 10
    torque_bins = np.linspace(torque_unscaled.min(), torque_unscaled.max(), n_bins)
    bin_indices = np.digitize(torque_unscaled, torque_bins)

    bin_maes = []
    bin_centers = []
    for i in range(1, len(torque_bins)):
        mask = bin_indices == i
        if np.sum(mask) > 10:  # At least 10 samples
            bin_mae = mean_absolute_error(y_true[mask], y_pred[mask])
            bin_maes.append(bin_mae)
            bin_centers.append((torque_bins[i - 1] + torque_bins[i]) / 2)

    plt.plot(bin_centers, bin_maes, "o-")
    plt.xlabel("Torque Range Center")
    plt.ylabel("Mean Absolute Error")
    plt.title("Performance by Torque Range")
    plt.grid(True, alpha=0.3)

    # Residuals vs predicted
    plt.subplot(2, 3, 6)
    plt.scatter(y_pred, errors, alpha=0.5, s=1)
    plt.axhline(y=0, color="r", linestyle="--")
    plt.xlabel("Predicted Throttle")
    plt.ylabel("Residuals")
    plt.title("Residuals vs Predicted")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(
        "tempres/inverse_controller_test_results.png", dpi=150, bbox_inches="tight"
    )
    plt.show()

    return {
        "predictions": y_pred,
        "actuals": y_true,
        "target_torques": torque_unscaled,
        "mse": mse,
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
    }


def predict_throttle_for_target(
    target_torque,
    current_state,
    model_path="tempres/best_inverse_torque_controller.pth",
    torque_scaler_path="tempres/torque_scaler.pkl",
    state_scaler_path="tempres/state_scaler.pkl",
):
    """
    Predict throttle input for a given target torque and current state

    Args:
        target_torque: Desired torque value
        current_state: Dict with keys ['RPM', 'gearRatio', 'engineLoad',
                      'wheelRR_speed', 'wheelRL_speed', 'brakeInput', 'steeringInput']
    """
    # Load scalers and model
    torque_scaler = joblib.load(torque_scaler_path)
    state_scaler = joblib.load(state_scaler_path)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = InverseTorqueController(len(current_state.keys())).to(device)  # Dynamically derive number of state features
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    # Prepare input
    target_array = np.array([[target_torque]], dtype=np.float32)
    state_array = np.array(
        [
            [
                current_state["RPM"],
                current_state["gearRatio"],
                current_state["engineLoad"],
                current_state["wheelRR_speed"],
                current_state["wheelRL_speed"],
                current_state["brakeInput"],
                current_state["steeringInput"],
            ]
        ],
        dtype=np.float32,
    )

    # Scale inputs
    target_scaled = torque_scaler.transform(target_array)
    state_scaled = state_scaler.transform(state_array)

    # Convert to tensors
    target_tensor = torch.from_numpy(target_scaled).to(device)
    state_tensor = torch.from_numpy(state_scaled).to(device)

    # Predict
    with torch.no_grad():
        predicted_throttle = model(target_tensor, state_tensor).cpu().numpy()[0, 0]

    return predicted_throttle


# Example usage
if __name__ == "__main__":
    # Test on one of your datasets
    test_data_path = (
        "/home/vincec4/Downloads/beamng_log_data/drive/run_003/data/data.pkl"
    )
    # test_data_path = "/home/vincec4/beamng_log_data/run_001/data/data.pkl"

    results = test_inverse_controller(test_data_path)

    # Example of using the controller for a specific target
    example_state = {
        "RPM": 2000,
        "gearRatio": 3.5,
        "engineLoad": 0.3,
        "wheelRR_speed": 15.0,
        "wheelRL_speed": 15.0,
        "brakeInput": 0.0,
        "steeringInput": 0.0,
    }

    target_torque = 500.0  # Desired torque
    predicted_throttle = predict_throttle_for_target(target_torque, example_state)
    print(f"\nExample prediction:")
    print(f"Target torque: {target_torque}")
    print(f"Predicted throttle: {predicted_throttle:.3f}")
