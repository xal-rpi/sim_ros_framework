#!/usr/bin/env python3
import argparse
import math

def generate_circle(radius: float, num_pts: int, closed: bool):
    """
    Generate points on a circle of radius `radius`.
    Points are at angles θₖ = 2π·k/N for k=0…N−1.
    If closed=True, the first point is appended at the end.
    """
    pts = []
    for k in range(num_pts):
        theta = 2.0 * math.pi * k / num_pts
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        pts.append((x, y))
    if closed and pts:
        pts.append(pts[0])
    return pts

def write_csv(points, filename: str):
    """Write list of (x,y) pairs to `filename` as CSV with two decimals."""
    with open(filename, "w") as f:
        for x, y in points:
            f.write(f"{x:.2f},{y:.2f}\n")

def main():
    p = argparse.ArgumentParser(
        description="Generate a circular path CSV of radius r with N points."
    )
    p.add_argument(
        "--radius",
        "-r",
        type=float,
        default=10.0,
        help="circle radius (default: 10.0)",
    )
    p.add_argument(
        "--num-points",
        "-n",
        type=int,
        default=8,
        help="number of waypoints around the circle (default: 8)",
    )
    p.add_argument(
        "--closed",
        "-c",
        action="store_true",
        help="repeat first point at end to close the loop",
    )
    p.add_argument(
        "--output",
        "-o",
        type=str,
        default="circle.csv",
        help="output CSV filename (default: circle.csv)",
    )

    args = p.parse_args()
    pts = generate_circle(args.radius, args.num_points, args.closed)
    write_csv(pts, args.output)
    print(f"Wrote {len(pts)} points to {args.output}")

if __name__ == "__main__":
    main()
