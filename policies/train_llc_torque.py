import joblib
import pickle
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import torch
from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt


# 1. Dataset class for inverse control
class InverseTorqueDataset(Dataset):
    def __init__(self, target_torques, state_features, throttles, weights):
        # target_torques: (N,1), state_features: (N,n), throttles: (N,1), weights: (N,1)
        self.target_torques = torch.from_numpy(target_torques)         # scaled
        self.state_features = torch.from_numpy(state_features)         # scaled
        self.throttles    = torch.from_numpy(throttles)
        self.weights      = torch.from_numpy(weights)                  # float32

    def __len__(self):
        return len(self.target_torques)

    def __getitem__(self, idx):
        return (self.target_torques[idx], self.state_features[idx], self.throttles[idx], self.weights[idx])


# 2. Inverse torque controller model
class InverseTorqueController(nn.Module):
    def __init__(self, n_state_features):
        super().__init__()
        # Input: [target_torque(1), state_features(n)]
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
            nn.Sigmoid(),  # throttle in [0,1]
        )

    def forward(self, target_torque, state_features):
        # Concatenate target torque with state features
        x = torch.cat([target_torque, state_features], dim=1)
        return self.net(x)


def load_and_prepare_data(data_paths, dataset_names):
    """Load and combine data from multiple sources"""
    all_data = []

    for path, name in zip(data_paths, dataset_names):
        print(f"Loading {name} from {path}...")

        with open(path, "rb") as f:
            data = pickle.load(f)

        state_dict = data[("ego", "gtstate")]
        df = pd.DataFrame(state_dict)

        # Add dataset identifier
        df["dataset"] = name
        all_data.append(df)
        print(f"  Loaded {len(df)} samples from {name}")

    # Combine all datasets
    combined_df = pd.concat(all_data, ignore_index=True)
    print(f"\nTotal combined samples: {len(combined_df)}")

    return combined_df


def prepare_inverse_training_data(df):
    """Prepare data for inverse torque control"""

    # State features that affect torque generation
    state_keys = [
        "RPM",
        "gearRatio",
        "engineLoad",
        "wheelRR_speed",
        "wheelRL_speed",
        "brakeInput",  # Important: brake affects available torque
        "steeringInput",  # May affect load
    ]

    # Target: achieved torque (average of rear wheels)
    df["avg_rear_torque"] = (df["wheelRR_propTorque"] + df["wheelRL_propTorque"]) / 2

    # Filter out invalid data
    # Remove samples where brake is applied (conflicting with throttle)
    df_clean = df[df["brakeInput"] < 0.1].copy()
    # Remove samples where torque is negative

    # Remove extreme outliers
    torque_99 = df_clean["avg_rear_torque"].quantile(0.99)
    df_clean = df_clean[
        (df_clean["avg_rear_torque"] >= 0)
        & (df_clean["avg_rear_torque"] <= torque_99)
    ].copy()

    print(f"After filtering: {len(df_clean)} samples")
    print(
        f"Torque range: {df_clean['avg_rear_torque'].min():.2f} to {df_clean['avg_rear_torque'].max():.2f}"
    )
    print(
        f"Throttle range: {df_clean['throttleInput'].min():.2f} to {df_clean['throttleInput'].max():.2f}"
    )

    # Extract arrays
    target_torques = df_clean[["avg_rear_torque"]].to_numpy(dtype=np.float32)
    state_features = df_clean[state_keys].to_numpy(dtype=np.float32)
    throttles = df_clean[["throttleInput"]].to_numpy(dtype=np.float32)

    return target_torques, state_features, throttles, state_keys


def main():
    # Data paths
    data_paths = [
        "/home/vincec4/beamng_log_data/run_001/data/data.pkl",  # Constant throttle
        "/home/vincec4/Downloads/beamng_log_data/drive_brake/run_001/data/data.pkl",  # Normal driving 1
        "/home/vincec4/Downloads/beamng_log_data/drive_brake/run_002/data/data.pkl",  # Normal driving 2
        "/home/vincec4/Downloads/beamng_log_data/drive/run_001/data/data.pkl",  # Normal driving 3
        "/home/vincec4/Downloads/beamng_log_data/drive/run_002/data/data.pkl",  # Normal driving 4
        # "/home/vincec4/Downloads/beamng_log_data/drive/run_003/data/data.pkl",  # Normal driving 5, testing
    ]

    dataset_names = ["constant_throttle","normal_driving_1", "normal_driving_2", "normal_driving_3", "normal_driving_4",] # "normal_driving_5"]

    # Load and combine data
    combined_df = load_and_prepare_data(data_paths, dataset_names)

    # Prepare training data
    target_torques, state_features, throttles, state_keys = prepare_inverse_training_data(combined_df)
    high_torque_thr = 800.0
    weights = np.ones_like(target_torques, dtype=np.float32)
    weights[target_torques[:,0] > high_torque_thr] = 5.0   # up‐weight 5× when torque>800 Nm

    print(f"\nTraining data shapes:")
    print(f"Target torques: {target_torques.shape}")
    print(f"State features: {state_features.shape}")
    print(f"Throttles: {throttles.shape}")
    print(f"State feature names: {state_keys}")

    # Split data
    indices = np.arange(len(target_torques))
    train_idx, val_idx = train_test_split(indices, test_size=0.2, random_state=42)

    target_train, target_val   = target_torques[train_idx], target_torques[val_idx]
    state_train, state_val     = state_features[train_idx], state_features[val_idx]
    throttle_train, throttle_val = throttles[train_idx], throttles[val_idx]
    weight_train, weight_val   = weights[train_idx], weights[val_idx]

    # Scale features
    # Scale target torques
    torque_scaler = StandardScaler()
    target_train_scaled = torque_scaler.fit_transform(target_train)
    target_val_scaled = torque_scaler.transform(target_val)

    # Scale state features
    state_scaler = StandardScaler()
    state_train_scaled = state_scaler.fit_transform(state_train)
    state_val_scaled = state_scaler.transform(state_val)

    # Save scalers
    joblib.dump(torque_scaler, "tempres/torque_scaler.pkl")
    joblib.dump(state_scaler, "tempres/state_scaler.pkl")

    # Create datasets and loaders
    train_ds = InverseTorqueDataset(
        target_train_scaled,
        state_train_scaled,
        throttle_train,
        weight_train.astype(np.float32).reshape(-1,1)
    )
    val_ds = InverseTorqueDataset(
        target_val_scaled,
        state_val_scaled,
        throttle_val,
        weight_val.astype(np.float32).reshape(-1,1)
    )

    train_loader = DataLoader(train_ds, batch_size=128, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=256, shuffle=False)

    # Model setup
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\nUsing device: {device}")

    model = InverseTorqueController(len(state_keys)).to(device)
    optimizer = optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    criterion = nn.HuberLoss()
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=10, factor=0.5)

    # Training loop
    train_losses = []
    val_losses = []
    best_val_loss = float("inf")
    patience_counter = 0

    print("\nStarting training...")

    for epoch in range(1, 501):
        # Training
        model.train()
        train_loss = 0.0
        for target_batch, state_batch, throttle_batch, weight_batch in train_loader:
            target_batch = target_batch.to(device)
            state_batch = state_batch.to(device)
            throttle_batch = throttle_batch.to(device)
            weight_batch  = weight_batch.to(device)

            optimizer.zero_grad()
            pred_throttle = model(target_batch, state_batch)
            loss = criterion(pred_throttle, throttle_batch)
            # loss = ((pred_throttle - throttle_batch)**2 * weight_batch).mean()
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * target_batch.size(0)

        train_loss /= len(train_ds)
        train_losses.append(train_loss)

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for target_batch, state_batch, throttle_batch, weight_batch in val_loader:
                target_batch = target_batch.to(device)
                state_batch = state_batch.to(device)
                throttle_batch = throttle_batch.to(device)

                pred_throttle = model(target_batch, state_batch)
                val_loss += criterion(
                    pred_throttle, throttle_batch
                ).item() * target_batch.size(0)

        val_loss /= len(val_ds)
        val_losses.append(val_loss)

        scheduler.step(val_loss)

        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), "tempres/best_inverse_torque_controller.pth")
            patience_counter = 0
        else:
            patience_counter += 1

        if epoch % 10 == 0:
            print(
                f"Epoch {epoch:03d} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f} | LR: {optimizer.param_groups[0]['lr']:.6f}"
            )

        # Early stopping
        if patience_counter >= 20:
            print(f"Early stopping at epoch {epoch}")
            break

    # Plot training curves
    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(train_losses, label="Train Loss")
    plt.plot(val_losses, label="Val Loss")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss")
    plt.title("Training Progress")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    plt.plot(train_losses[-50:], label="Train Loss (Last 50)")
    plt.plot(val_losses[-50:], label="Val Loss (Last 50)")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss")
    plt.title("Training Progress (Last 50 Epochs)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("tempres/training_progress.png", dpi=150, bbox_inches="tight")
    plt.show()

    print(f"\nTraining completed!")
    print(f"Best validation loss: {best_val_loss:.6f}")
    print(f"Model saved to: tempres/best_inverse_torque_controller.pth")


if __name__ == "__main__":
    main()
