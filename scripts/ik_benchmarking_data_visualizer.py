#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import rclpy
from rclpy.node import Node


class DataVisualizerNode(Node):
    def __init__(self):
        super().__init__("data_visualizer_node")

        # Allow loading the IK benchmarking data from non-current directories
        # using a 'data_directory' parameter that can be passed when running the script
        # the 'data_directory' parameter defaults to current directory, and if no data exist, a warning is printed
        self.declare_parameter("data_directory", os.getcwd())
        self.data_directory = (
            self.get_parameter("data_directory").get_parameter_value().string_value
        )
        print(f"{'=' * 60}")
        print(
            f"\nThe benchmarking CSV files will be loaded from the directory:\n\n{self.data_directory}"
        )

        self.run_visualization()

    def run_visualization(self):
        data_list = self.read_ik_benchmarking_files()

        # Check if the data files really exist
        if not data_list:
            self.get_logger().warn(
                f"No IK benchmarking CSV data files found in the directory: {self.data_directory}"
            )
            print(f"{'=' * 60}")

            rclpy.shutdown()
            return

        self.plot_data(data_list)
        self.write_summary_table(data_list)

    def read_ik_benchmarking_files(self):
        file_pattern = os.path.join(self.data_directory, "*ik_benchmarking_data.csv")
        files = glob.glob(file_pattern)
        data_list = []

        for file in files:
            data = pd.read_csv(file)

            # found_ik is written via std::boolalpha as "true"/"false" strings.
            # Coerce to actual bool so it can be used as a row mask downstream.
            if data["found_ik"].dtype == object:
                data["found_ik"] = (
                    data["found_ik"]
                    .astype(str)
                    .str.strip()
                    .str.lower()
                    .map({"true": True, "false": False})
                )

            # The server opens the CSV in std::ios::app mode, so re-running a solver
            # appends another header + 10k rows to the existing file. Drop any rows
            # whose found_ik did not parse to a real bool (those are stray header rows
            # from previous runs leaking in).
            before = len(data)
            data = data.dropna(subset=["found_ik"]).reset_index(drop=True)
            dropped = before - len(data)
            if dropped:
                print(
                    f"[{os.path.basename(file)}] dropped {dropped} stray header rows "
                    f"from accumulated previous runs; keeping {len(data)} trials."
                )
            data["found_ik"] = data["found_ik"].astype(bool)

            # Convert solve_time and errors to numeric
            for col in ["solve_time", "position_error", "orientation_error"]:
                data[col] = pd.to_numeric(data[col], errors="coerce")
            # Process the filename and remove the common suffix
            file_label = os.path.basename(file).replace("_ik_benchmarking_data.csv", "")
            data_list.append((file_label, data))

        return data_list

    def plot_data(self, data_list):
        print(f"\nBenchmarking result plots will be saved in the same directory.\n")
        print(f"{'=' * 60}")

        # Box plot for solve times of successful trials
        plt.figure(figsize=(15, 10))

        all_data = []
        labels = []

        for file, data in data_list:
            success_data = data[data["found_ik"]]
            all_data.extend(success_data.dropna(subset=["solve_time"])["solve_time"])
            labels.extend([file] * len(success_data))
        df_for_boxplot = pd.DataFrame({"Solve Times": all_data, "Dataset": labels})

        # Box plot for solve times
        sns.boxplot(
            x="Dataset",
            y="Solve Times",
            data=df_for_boxplot,
            showfliers=False,
            boxprops={"edgecolor": "black"},
            palette="Blues",
        )
        plt.title("Solve Times for Successful Trials")
        plt.ylabel("Microseconds")
        plt.xlabel("IK Solvers")
        plt.savefig(os.path.join(self.data_directory, "solve_times.png"))

        # Bar chart for success rates
        success_rates = [
            (f'{file}\n{100 * data["found_ik"].mean():.2f}%', data["found_ik"].mean())
            for file, data in data_list
        ]
        labels, rates = zip(*success_rates)
        df_for_barplot = pd.DataFrame({"Success Rates": rates, "Dataset": labels})
        plt.figure(figsize=(15, 10))
        sns.barplot(
            x="Dataset",
            y="Success Rates",
            data=df_for_barplot,
            edgecolor="black",
            palette="Blues",
        )
        plt.ylim(0, 1)
        plt.title("Success Rate for Each Dataset")
        plt.ylabel("Rate")
        plt.xlabel("IK Solvers")
        plt.savefig(os.path.join(self.data_directory, "success_rates.png"))

        # Box plot for position_error, and orientation_error
        error_types = [("position_error", "Meters"), ("orientation_error", "Radians")]

        for error_type, unit in error_types:
            plt.figure(figsize=(15, 10))
            all_error_data = []
            error_labels = []

            for file, data in data_list:
                success_data = data[data["found_ik"]]
                all_error_data.extend(
                    success_data.dropna(subset=[error_type])[error_type]
                )
                error_labels.extend([file] * len(success_data))

            df_error_for_plot = pd.DataFrame(
                {error_type: all_error_data, "Dataset": error_labels}
            )
            sns.boxplot(
                x="Dataset",
                y=error_type,
                data=df_error_for_plot,
                showfliers=False,
                boxprops={"edgecolor": "black"},
                palette="Blues",
            )
            plt.title(f'{error_type.replace("_", " ").title()} for Successful Trials')
            plt.ylabel(f'{error_type.replace("_", " ").title()} ({unit})')
            plt.xlabel("IK Solvers")
            plt.savefig(os.path.join(self.data_directory, f"{error_type}.png"))

    def write_summary_table(self, data_list):
        """Aggregate per-solver statistics. Print to stdout, save as summary.csv
        (machine-readable, SI units) and summary.md (markdown, human-readable)."""

        def stats(series):
            s = series.dropna()
            if s.empty:
                return {"mean": float("nan"), "median": float("nan"),
                        "p95": float("nan"), "max": float("nan")}
            return {
                "mean": float(s.mean()),
                "median": float(s.median()),
                "p95": float(s.quantile(0.95)),
                "max": float(s.max()),
            }

        rows = []
        for label, data in sorted(data_list, key=lambda x: x[0]):
            success_mask = data["found_ik"]
            n_trials = int(len(data))
            n_success = int(success_mask.sum())
            success_rate = n_success / n_trials if n_trials else float("nan")
            success_only = data[success_mask]
            t = stats(success_only["solve_time"])           # microseconds
            p = stats(success_only["position_error"])       # meters
            o = stats(success_only["orientation_error"])    # radians
            rows.append({
                "solver": label,
                "trials": n_trials,
                "success": n_success,
                "success_rate": success_rate,
                "solve_us_mean": t["mean"],
                "solve_us_median": t["median"],
                "solve_us_p95": t["p95"],
                "solve_us_max": t["max"],
                "pos_err_m_mean": p["mean"],
                "pos_err_m_median": p["median"],
                "pos_err_m_p95": p["p95"],
                "orient_err_rad_mean": o["mean"],
                "orient_err_rad_median": o["median"],
                "orient_err_rad_p95": o["p95"],
            })

        summary = pd.DataFrame(rows)

        # 1) summary.csv — raw SI numbers, machine-readable.
        csv_path = os.path.join(self.data_directory, "summary.csv")
        summary.to_csv(csv_path, index=False)

        # 2) summary.md and stdout — human-readable, with friendly units.
        display = pd.DataFrame({
            "solver": summary["solver"],
            "trials": summary["trials"].astype(int),
            "success": summary.apply(
                lambda r: f'{int(r["success"])} ({100 * r["success_rate"]:.2f}%)', axis=1
            ),
            "solve_us (mean/med/p95)": summary.apply(
                lambda r: f'{r["solve_us_mean"]:.0f} / {r["solve_us_median"]:.0f} / {r["solve_us_p95"]:.0f}',
                axis=1,
            ),
            "pos_err_mm (mean/p95)": summary.apply(
                lambda r: f'{1000*r["pos_err_m_mean"]:.4f} / {1000*r["pos_err_m_p95"]:.4f}',
                axis=1,
            ),
            "orient_err_deg (mean/p95)": summary.apply(
                lambda r: f'{180/3.141592653589793*r["orient_err_rad_mean"]:.4f} / '
                          f'{180/3.141592653589793*r["orient_err_rad_p95"]:.4f}',
                axis=1,
            ),
        })

        md_path = os.path.join(self.data_directory, "summary.md")
        try:
            md_table = display.to_markdown(index=False)
        except ImportError:
            # tabulate not installed — fall back to plain string table.
            md_table = display.to_string(index=False)
        with open(md_path, "w") as fh:
            fh.write("# IK benchmarking summary\n\n")
            fh.write(md_table)
            fh.write("\n\nGenerated from CSVs in " + self.data_directory + "\n")

        print("\nPer-solver summary (mm/deg shown for errors):\n")
        print(display.to_string(index=False))
        print(f"\nSaved {csv_path} and {md_path}")
        print(f"{'=' * 60}")


if __name__ == "__main__":
    rclpy.init(args=None)
    node = DataVisualizerNode()
    if rclpy.ok():
        rclpy.shutdown()
