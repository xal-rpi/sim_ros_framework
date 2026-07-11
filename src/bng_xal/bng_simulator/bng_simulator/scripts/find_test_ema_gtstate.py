#!/usr/bin/env python3
"""
Inspect GTState pickle files with EMA preview, jitter detection, and interactive
alpha‐tuning for the top‐N jittery fields, all plotted against the 'time' field.
"""
import os
import csv
import pickle
import argparse

import numpy as np

try:
    import pandas as pd
except ImportError:
    pd = None

try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button
except ImportError:
    plt = None


def load_pickle(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def compute_jitter_metrics(df, alpha_j):
    eps = 1e-8
    metrics = []
    for col in df.columns:
        if col == "time" or not pd.api.types.is_numeric_dtype(df[col]):
            continue
        s = df[col].dropna()
        if s.size < 2:
            continue
        ema = s.ewm(alpha=alpha_j).mean()
        res = s - ema
        sigma_res = res.std()
        sigma_x = s.std()
        J = sigma_res / (sigma_x + eps)
        metrics.append((col, J, float(sigma_res), float(sigma_x)))
    metrics.sort(key=lambda x: x[1], reverse=True)
    return metrics


def print_jitter_metrics(metrics, top_n, threshold):
    print(f"\nTop {top_n} jittery fields (J = σ_res/σ_x):")
    for i, (col, J, sr, sx) in enumerate(metrics[:top_n]):
        flag = "⚠️" if J >= threshold else ""
        print(
            f"  {i+1:2d}. {col:20s}  J={J:.3f}  " f"σ_res={sr:.3f}  σ_x={sx:.3f} {flag}"
        )


def preview_saved_ema(df, saved_map, do_plot=True):
    """
    For each (field→α) in saved_map, compute EMA and plot orig vs EMA.
    """
    for fld, α in saved_map.items():
        if fld not in df.columns:
            print(f"!! field {fld!r} not in data, skipping")
            continue

        print(f"\n--- Replay EMA for {fld!r} with α={α:.2f} ---")
        sub = df[["time", fld]] if "time" in df.columns else df[[fld]]
        sub = sub.dropna()
        x = sub["time"].values if "time" in sub.columns else np.arange(len(sub))
        y = sub[fld].values

        ema = pd.Series(y).ewm(alpha=α).mean().values
        print(f" head orig = {y[:5].tolist()}\n head ema  = {ema[:5].tolist()}")

        if do_plot:
            plt.figure(figsize=(6, 3))
            plt.title(f"{fld!r} (α={α:.2f})")
            plt.plot(x, y, label="orig", lw=1)
            plt.plot(x, ema, label="EMA", lw=2)
            plt.xlabel("time" if "time" in df.columns else "index")
            plt.legend()
            plt.tight_layout()
            plt.show()


def preview_ema(df, fields, alphas, do_plot):
    """
    Static EMA preview, plotted vs. 'time' if available.
    """
    for fld in fields:
        if fld not in df.columns:
            print(f"!! field {fld!r} not found, skipping")
            continue
        print(f"\n--- EMA preview for field {fld!r} ---")
        x = df["time"].values if "time" in df.columns else np.arange(len(df))
        if do_plot and plt:
            plt.figure(figsize=(6, 3))
            plt.title(f"Field: {fld}")
            plt.plot(x, df[fld].values, label="orig", lw=1)
        for α in alphas:
            ema = df[fld].ewm(alpha=α).mean()
            print(f" α={α:.2f} head={ema.head().tolist()}")
            if do_plot and plt:
                plt.plot(x, ema.values, label=f"α={α:.2f}", lw=1)
        if do_plot and plt:
            plt.xlabel("time" if "time" in df.columns else "index")
            plt.legend()
            plt.tight_layout()
            plt.show()


def interactive_plot_field(df, field, init_alpha, out_csv):
    """
    Interactive plot vs. 'time', slider for α, Save/Next buttons.
    """
    if plt is None:
        print("matplotlib required for interactive mode")
        return

    # extract time & field, drop any NaNs
    sub = df[["time", field]].dropna()
    x = sub["time"].values
    y = sub[field].values

    fig, ax = plt.subplots(figsize=(8, 4))
    plt.subplots_adjust(bottom=0.25, top=0.85)
    ax.set_title(f"Field: {field}")
    ax.set_xlabel("time")
    ax.plot(x, y, label="orig", lw=1)

    ema_vals = pd.Series(y).ewm(alpha=init_alpha).mean().values
    (ema_line,) = ax.plot(x, ema_vals, label=f"EMA α={init_alpha:.2f}", lw=2)
    ax.legend()

    # Slider for α
    ax_slider = fig.add_axes([0.25, 0.10, 0.50, 0.03])
    slider = Slider(ax_slider, "α", 0.0, 1.0, valinit=init_alpha, valstep=0.01)

    # Save and Next buttons
    ax_save = fig.add_axes([0.80, 0.025, 0.10, 0.04])
    btn_save = Button(ax_save, "Save")
    ax_next = fig.add_axes([0.65, 0.025, 0.10, 0.04])
    btn_next = Button(ax_next, "Next")

    def update(val):
        a = slider.val
        new_ema = pd.Series(y).ewm(alpha=a).mean().values
        ema_line.set_ydata(new_ema)
        ema_line.set_label(f"EMA α={a:.2f}")
        ax.legend()
        fig.canvas.draw_idle()

    def save(event):
        a = slider.val
        new_file = not os.path.isfile(out_csv)
        with open(out_csv, "a", newline="") as cf:
            w = csv.writer(cf)
            if new_file:
                w.writerow(["field", "alpha"])
            w.writerow([field, f"{a:.2f}"])
        print(f"Saved: {field} α={a:.2f} → {out_csv}")

    def nxt(event):
        plt.close(fig)

    slider.on_changed(update)
    btn_save.on_clicked(save)
    btn_next.on_clicked(nxt)

    plt.show()


def interactive_jitter(df, metrics, args):
    jitter_fields = [col for col, *_ in metrics[: args.top_n]]
    print(f"\nInteractive tuning for fields: {jitter_fields}")
    for fld in jitter_fields:
        interactive_plot_field(df, fld, args.jitter_alpha, args.output_csv)


def process_file(path, args):
    print(f"\nLoading {path!r} …")
    data = load_pickle(path)
    if "data" in data:
        data = data["data"]
    for (veh, sensor), fld_dict in data.items():
        for k, v in fld_dict.items():
            print(f"  - {k:20s} : {len(v)} samples")
        if pd is None:
            print("pandas required; skipping.")
            continue

        df = pd.DataFrame(fld_dict)

        # EMA preview / interactive tuning for --fields
        if args.fields:
            if args.interactive:
                # for each requested field, pop up the slider/save UI
                for fld in args.fields:
                    if fld not in df.columns:
                        print(f"!! field {fld!r} not found, skipping")
                        continue
                    # pick an initial α (here the first of --alphas)
                    init_alpha = args.alphas[0]
                    print(
                        f"\nInteractive tuning for field {fld!r} (init α={init_alpha})"
                    )
                    interactive_plot_field(df, fld, init_alpha, args.output_csv)
            else:
                # static preview only
                preview_ema(df, args.fields, args.alphas, do_plot=not args.no_plot)

        # jitter detect / interactive tuning for jittery fields
        if args.detect_jitter:
            metrics = compute_jitter_metrics(df, args.jitter_alpha)
            if args.interactive:
                interactive_jitter(df, metrics, args)
            else:
                print_jitter_metrics(metrics, args.top_n, args.jitter_threshold)


def main():
    p = argparse.ArgumentParser(
        description="GTState pickle viewer with interactive EMA tuning"
    )
    p.add_argument("-d", "--dir", help="directory containing data_*.pkl files")
    p.add_argument("-f", "--file", help="specific pickle file to load")
    p.add_argument(
        "-F", "--fields", nargs="+", default=[], help="fields to EMA-preview"
    )
    p.add_argument(
        "-a",
        "--alphas",
        nargs="+",
        type=float,
        default=[0.1, 0.3, 0.6],
        help="EMA α values for preview",
    )
    p.add_argument(
        "--detect-jitter", action="store_true", help="compute & rank jitter metrics"
    )
    p.add_argument(
        "--jitter-alpha",
        type=float,
        default=0.2,
        help="fast EMA α for jitter residual",
    )
    p.add_argument("--top-n", type=int, default=5, help="top‐N jittery fields to show")
    p.add_argument(
        "--jitter-threshold",
        type=float,
        default=0.1,
        help="flag fields with J ≥ thresh",
    )
    p.add_argument(
        "--interactive",
        action="store_true",
        help="interactive alpha tuning for jitter fields",
    )
    p.add_argument(
        "--no-plot", action="store_true", help="suppress static matplotlib plots"
    )
    p.add_argument(
        "--use-csv",
        default=None,
        help="plot original vs EMA using saved α from --output-csv",
    )
    p.add_argument(
        "-o",
        "--output-csv",
        default="ema_alphas.csv",
        help="CSV file to save tuned alphas",
    )
    args = p.parse_args()

    paths = []
    if args.file:
        paths = [args.file]
    elif args.dir:
        paths = sorted(
            os.path.join(args.dir, fn)
            for fn in os.listdir(args.dir)
            if fn.endswith(".pkl")
        )
    if not paths:
        print("No pickle files found. Use --file or --dir.")
        return
    # If user just wants to replay saved αs against data
    if args.use_csv:
        if pd is None or plt is None:
            print("ERROR: pandas+matplotlib required for --use-csv")
            return

        saved = {}
        try:
            with open(args.use_csv, newline="") as cf:
                r = csv.DictReader(cf)
                for row in r:
                    saved[row["field"]] = float(row["alpha"])
        except FileNotFoundError:
            print(f"ERROR: cannot open {args.output_csv!r}")
            return

        # for each pickle, plot each saved field
        for pth in (
            [args.file]
            if args.file
            else sorted(
                os.path.join(args.dir, fn)
                for fn in os.listdir(args.dir)
                if fn.endswith(".pkl")
            )
        ):
            print(f"\n→ Loading {pth!r}")
            data = load_pickle(pth)
            if "data" in data:
                data = data["data"]
            for (veh, sensor), fld_dict in data.items():
                print(f"\n=== Vehicle:{veh!r} Sensor:{sensor!r} ===")
                df = pd.DataFrame(fld_dict)
                preview_saved_ema(df, saved, do_plot=not args.no_plot)
        return

    if args.interactive and plt is None:
        print("ERROR: matplotlib is required for interactive mode.")
        return

    for pth in paths:
        try:
            process_file(pth, args)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"ERROR processing {pth!r}: {e}")


if __name__ == "__main__":
    main()
