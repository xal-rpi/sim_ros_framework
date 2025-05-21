#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd

def generate_loop_multiscale(
    num_points: int,
    length: float,
    noise_long: float,
    smooth_long: float,
    noise_short: float,
    smooth_short: float
):
    """
    Build a closed loop of perimeter `length` m with `num_points` samples,
    combining two Fourier‐noise bands:
    - Long‐scale wiggles of amplitude `noise_long` and wavelength ~`smooth_long`
    - Short‐scale wiggles of amplitude `noise_short` and wavelength ~`smooth_short`
    """
    R0 = length / (2 * np.pi)
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=True)

    # helper to build one band of modes
    def band(noise, smooth_len):
        K = max(1, min(num_points//2, int(length / smooth_len)))
        decay = K / 3.0
        δr = np.zeros_like(theta)
        for k in range(1, K+1):
            A = noise * np.exp(-k/decay)
            φ = np.random.uniform(0, 2*np.pi)
            δr += A * np.cos(k*theta + φ)
        return δr

    δr = band(noise_long, smooth_long) + band(noise_short, smooth_short)
    r = np.maximum(R0 + δr, 0.1)

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # global rescale so perimeter == length
    dx = np.diff(np.append(x, x[0]))
    dy = np.diff(np.append(y, y[0]))
    perim = np.hypot(dx, dy).sum()
    scale = length / perim
    return x * scale, y * scale

def main():
    p = argparse.ArgumentParser(
        description="Generate a multi-scale looping random path and save as CSV"
    )
    p.add_argument("-n", "--num_points", type=int, default=500,
                   help="Number of vertices (default: 500)")
    p.add_argument("-l", "--length", type=float, default=200.0,
                   help="Total path length in metres (default: 200)")
    p.add_argument("--noise_long", type=float, default=3.0,
                   help="Long-scale radial σ in metres (default: 3)")
    p.add_argument("--smooth_long", type=float, default=40.0,
                   help="Long-scale wavelength in metres (default: 40)")
    p.add_argument("--noise_short", type=float, default=1.0,
                   help="Short-scale radial σ in metres (default: 1)")
    p.add_argument("--smooth_short", type=float, default=6.0,
                   help="Short-scale wavelength in metres (default: 6)")
    p.add_argument("-o", "--output", type=str, default="path.csv",
                   help="Output CSV filename (default: path.csv)")
    args = p.parse_args()

    x, y = generate_loop_multiscale(
        args.num_points, args.length,
        args.noise_long, args.smooth_long,
        args.noise_short, args.smooth_short
    )
    df = pd.DataFrame({"x": x, "y": y})
    df["x"] += df["x"].max()
    df.to_csv(args.output, index=False, header=False)
    print(
        f"Wrote loop of length {args.length:.1f} m with {args.num_points} pts\n"
        f"  • long-scale: σ={args.noise_long:.1f} m @ λ~{args.smooth_long:.1f} m\n"
        f"  • short-scale: σ={args.noise_short:.1f} m @ λ~{args.smooth_short:.1f} m\n"
        f"→ {args.output}"
    )

if __name__ == "__main__":
    main()
