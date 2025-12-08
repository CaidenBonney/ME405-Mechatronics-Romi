from matplotlib import pyplot as plt
from pathlib import Path
from math import log, pi
import numpy as np


def reject(reason, linenum, line):
    print(f"Line {linenum} rejected due to {reason}: {line}")


def process_data(f_name, fig, ax, column1_idx, column2_idx, effort, tau_K):

    # Variable initialization
    file_name = f_name
    col_number = []
    column1 = []
    column2 = []
    first = True

    # Open the file in read mode
    with open(file_name, "r") as f:
        content = [l.strip() for l in f.read().splitlines()]

    for linenum, line in enumerate(content, start=1):
        # Skip empty lines
        if not line:
            reject("empty line", linenum, line)
        else:
            # Skip full-line comments
            if line[0] == "#":
                reject("full-line comment", linenum, line)
            else:
                # Remove in-line comments
                line_no_comments = line.split("#")[0]

                # Split lines at comma and strip whitespace
                splitline = [s.strip() for s in line_no_comments.split(",")]

                # Line only valid if containing at least six entries
                if len(splitline) < 6:
                    reject("not enough entries", linenum, line)
                else:
                    # First valid line is header
                    if first:
                        first = False
                        column_headers = [
                            splitline[0],
                            splitline[1],
                            splitline[2],
                            splitline[3],
                            splitline[4],
                            splitline[5],
                        ]

                    else:
                        # Convert first 2 entries to floats and add to respective columns
                        try:
                            entries = [
                                float(splitline[0]),
                                float(splitline[1]),
                                float(splitline[2]),
                                float(splitline[3]),
                                float(splitline[4]),
                                float(splitline[5]),
                            ]
                            # col_number.append(linenum) # line number also added for debugging if needed
                            column1.append(entries[column1_idx])  # this needs to be depend on which test
                            column2.append(entries[column2_idx])  # this needs to be depend on which test

                        except ValueError:
                            reject("invalid value type", linenum, line)

    gear_ratio = 119.7576  # shaft revolution / wheel revolution
    counts_per_rev = 12  # ticks / shaft revolution
    wheel_radius = 35  # mm

    # CONDITION DATA

    column1 = [x / 1e6 for x in column1]  # change us to seconds
    steady_state = 0

    column2 = [x * 2 * pi / (counts_per_rev * gear_ratio) for x in column2]  # change ticks to radians
    if column2_idx == 4 or column2_idx == 5:
        column2 = [x * wheel_radius for x in column2]  # change rad/us to translational mm/s of wheel
        steady_state = sum(column2[-10:]) / 10  # steady state velocity

        # remove values above 0.5 sec
        keepx = [i for i, x in enumerate(column1) if x <= 0.5]
        column1 = [column1[i] for i in keepx]
        column2 = [column2[i] for i in keepx]

        # remove values that are above steady state
        keepy = [i for i, y in enumerate(column2) if y < steady_state]
        column1 = [column1[i] for i in keepy]
        column2 = [log(1 - column2[i] / steady_state) for i in keepy]

    # TIME CONSTANT CALCS
    x = np.array(column1)  # or: x = np.array([ ... ])
    y = np.array(column2)

    # 1) Fit line of best fit (least squares)
    slope, intercept = np.polyfit(x, y, 1)  # degree=1 → line

    # # 3) Plot best-fit line across the data range
    # x_fit = np.linspace(x.min(), x.max(), 200)
    # y_fit = slope * x_fit + intercept
    tau = (slope**-1) * -1

    # STEADY STATE GAIN CALCS
    V_applied = 8.4
    k = steady_state / (effort / 100 * V_applied) / wheel_radius

    tau_K.append((effort, tau, k))

    ax.plot(column1, column2, label=f"effort = {effort}, tau = {tau:.2f} s")


left_right_ave = []


def multifile_process_data(file_num_list, folder_name, fig, ax, column1_idx, column2_idx):
    tau_K = []
    for i in file_num_list:
        process_data(f"{folder_name}log{i}.csv", fig, ax, column1_idx, column2_idx, i * 10 if i != 0 else 100, tau_K)
    # for i in tau_K:
    #     print(f"pwm = {i[0]}, tau = {i[1]:.2f} s, k = {i[2]:.2f}")
    print(f"average tau: {sum(t[1] for t in tau_K) / len(tau_K)}")
    print(f"average K: {sum(t[2] for t in tau_K) / len(tau_K)}")
    left_right_ave.append(
        (sum(t[1] for t in tau_K) / len(tau_K), sum(t[2] for t in tau_K) / len(tau_K))
    )  # tau_ave, K_ave


def main(results_folder):
    file_num_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0]

    # ---------------- plots left tau ----------------------
    ltau_fig, ltau_ax = plt.subplots()
    ltau_ax.set_title("Left: ln(1-Vact/Vset) vs. time")
    ltau_ax.set_xlabel("time, t (s)")
    ltau_ax.set_ylabel("ln(1-Vact/Vset) ")

    print("Left: ")
    multifile_process_data(file_num_list, results_folder, ltau_fig, ltau_ax, 0, 4)

    ltau_ax.legend(loc="upper right")
    ltau_fig.tight_layout()
    script_dir = Path(__file__).resolve().parent
    svg_path = script_dir / f"left_taus.svg"
    ltau_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- plots right tau ----------------------
    rtau_fig, rtau_ax = plt.subplots()
    rtau_ax.set_title("Right: ln(1-Vact/Vset) vs. time")
    rtau_ax.set_xlabel("time, t (s)")
    rtau_ax.set_ylabel("ln(1-Vact/Vset) ")

    print("Right: ")
    multifile_process_data(file_num_list, results_folder, rtau_fig, rtau_ax, 0, 5)

    ltau_ax.legend(loc="upper right")
    ltau_fig.tight_layout()
    script_dir = Path(__file__).resolve().parent
    svg_path = script_dir / f"right_taus.svg"
    ltau_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- total ave tau and K ----------------------
    print(f"Total average tau: {sum(t[0] for t in left_right_ave) / len(left_right_ave)}")
    print(f"Total average K: {sum(t[1] for t in left_right_ave) / len(left_right_ave)}")

    # # ---------------- plots right theta ----------------------
    # rtheta_fig, rtheta_ax = plt.subplots()
    # rtheta_ax.set_title("right theta vs. time")
    # rtheta_ax.set_xlabel("time, t (us)")
    # rtheta_ax.set_ylabel("right wheel theta, rtheta (ticks)")

    # multifile_process_data(file_num_list, results_folder, rtheta_fig, rtheta_ax, 1, 3)

    # rtheta_ax.legend(loc= "upper left")
    # rtheta_fig.tight_layout()
    # script_dir = Path(__file__).resolve().parent
    # svg_path = script_dir / f"right_theta.svg"
    # rtheta_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # # ---------------- plots left velocity ----------------------
    # lv_fig, lv_ax = plt.subplots()
    # lv_ax.set_title("left velocity vs. time")
    # lv_ax.set_xlabel("time, t (us)")
    # lv_ax.set_ylabel("left wheel velocity, lv (ticks/s)")

    # multifile_process_data(file_num_list, results_folder, lv_fig, lv_ax, 0, 4)

    # lv_ax.legend(loc= "lower right")
    # lv_fig.tight_layout()
    # script_dir = Path(__file__).resolve().parent
    # svg_path = script_dir / f"left_velocity.svg"
    # lv_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # # ---------------- plots right velocity ----------------------
    # rv_fig, rv_ax = plt.subplots()
    # rv_ax.set_title("right velocity vs. time")
    # rv_ax.set_xlabel("time, t (us)")
    # rv_ax.set_ylabel("right wheel velocity, rv (tick/s)")

    # multifile_process_data(file_num_list, results_folder, rv_fig, rv_ax, 1, 5)

    # rv_ax.legend(loc= "lower right")
    # rv_fig.tight_layout()
    # svg_path = script_dir / f"right_velocity.svg"
    # rv_fig.savefig(svg_path, format="svg", bbox_inches="tight")


if __name__ == "__main__":
    results_folder = "./Lab 0x05/Results/Data Floor 2 sec (from Lab 0x02)/"
    main(results_folder)
