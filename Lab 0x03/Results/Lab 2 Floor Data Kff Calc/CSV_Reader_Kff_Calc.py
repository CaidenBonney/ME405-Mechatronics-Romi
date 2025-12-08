from cycler import K
from matplotlib import pyplot as plt
from pathlib import Path
import numpy as np


def reject(reason, linenum, line):
    print(f"Line {linenum} rejected due to {reason}: {line}")


omega_ss_l = []
omega_ss_r = []


def process_data(f_name, fig, ax, column1_idx, column2_idx, effort):

    # Variable initialization
    file_name = f_name
    col_number = []
    column_x = []
    column_y = []
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
                            column_x.append(entries[column1_idx])  # this needs to be depend on which test
                            column_y.append(entries[column2_idx])  # this needs to be depend on which test

                        except ValueError:
                            reject("invalid value type", linenum, line)
        
    ax.plot(column_x, column_y, label=f"Effort = {effort}")
    
    if column2_idx == 4:
        omega_ss_l.append((effort, np.mean(column_y[-20:])*2*np.pi*35*1e6/(12*119.7576)))
    elif column2_idx == 5:
        omega_ss_r.append((effort, np.mean(column_y[-20:])*2*np.pi*35*1e6/(12*119.7576)))


def multifile_process_data(file_num_list, script_dir, fig, ax, column1_idx, column2_idx):
    for i in file_num_list:
        process_data(f"{script_dir}/log{i}.csv", fig, ax, column1_idx, column2_idx, i * 10 if i != 0 else 100)


def main():
    file_num_list = [1,2,3,4,5,6,7,8,9,0]
    file_num_list.reverse()
    script_dir = Path(__file__).resolve().parent

    # ---------------- plots left theta ----------------------
    ltheta_fig, ltheta_ax = plt.subplots()
    ltheta_ax.set_title("left theta vs. time")
    ltheta_ax.set_xlabel("time, t (us)")
    ltheta_ax.set_ylabel("left wheel theta, ltheta (rad)")

    multifile_process_data(file_num_list, script_dir, ltheta_fig, ltheta_ax, 0, 2)

    ltheta_ax.legend(loc="upper left")
    ltheta_fig.tight_layout()
    svg_path = script_dir / f"left_theta.svg"
    ltheta_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- plots right theta ----------------------
    rtheta_fig, rtheta_ax = plt.subplots()
    rtheta_ax.set_title("right theta vs. time")
    rtheta_ax.set_xlabel("time, t (us)")
    rtheta_ax.set_ylabel("right wheel theta, rtheta (rad)")

    multifile_process_data(file_num_list, script_dir, rtheta_fig, rtheta_ax, 1, 3)

    rtheta_ax.legend(loc="upper left")
    rtheta_fig.tight_layout()
    svg_path = script_dir / f"right_theta.svg"
    rtheta_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- plots left velocity ----------------------
    lv_fig, lv_ax = plt.subplots()
    lv_ax.set_title("left velocity vs. time")
    lv_ax.set_xlabel("time, t (us)")
    lv_ax.set_ylabel("left wheel velocity, lv (mm/s)")

    multifile_process_data(file_num_list, script_dir, lv_fig, lv_ax, 0, 4)

    lv_ax.legend(loc="lower right")
    lv_fig.tight_layout()
    svg_path = script_dir / f"left_velocity.svg"
    lv_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- plots right velocity ----------------------
    rv_fig, rv_ax = plt.subplots()
    rv_ax.set_title("right velocity vs. time")
    rv_ax.set_xlabel("time, t (us)")
    rv_ax.set_ylabel("right wheel velocity, rv (mm/s)")

    multifile_process_data(file_num_list, script_dir, rv_fig, rv_ax, 1, 5)

    rv_ax.legend(loc="lower right")
    rv_fig.tight_layout()
    svg_path = script_dir / f"right_velocity.svg"
    rv_fig.savefig(svg_path, format="svg", bbox_inches="tight")

    # ---------------- plots omega ss vs effort ----------------------
    # Example data
    omega_ss_lz = [(x, y / 1e6) for x, y in omega_ss_l]
    x_l = [x for x, _ in omega_ss_lz]
    y_l = [y for _, y in omega_ss_lz]
    m_l, b_l = np.polyfit(x_l, y_l, 1)
    # print(f"m = {m_l}")
    # print(f"b = {b_l}")
    # print(f"Equation: y = {m_l:.3f}x + {b_l:.3f}")

    # Example data
    omega_ss_rz = [(x, y / 1e6) for x, y in omega_ss_r]
    x_r = [x[0] for x in omega_ss_rz]
    y_r = [x[1] for x in omega_ss_rz]
    m_r, b_r = np.polyfit(x_r, y_r, 1)
    # print(f"m = {m_r}")
    # print(f"b = {b_r}")
    # print(f"Equation: y = {m_r:.3f}x + {b_r:.3f}")

    # Make figure
    o_fig, o_ax = plt.subplots()
    o_ax.set_title("Steady State Wheel Velocity vs. Pulse Width")
    o_ax.set_xlabel("Pulse Width (%)")
    o_ax.set_ylabel("Wheel Velocity, v (mm/s)")

    # Make plots
    o_ax.plot(x_l, y_l, marker='o', label=f"Left Wheel: y={m_l:.3f}x+{b_l:.3f}")
    # print slope and intercept
    Kff_l = 1/m_l
    print(f"Kff_l = {Kff_l}")
    print(f"ff_start_l = {-b_l/m_l}")
    o_ax.plot(x_r, y_r, marker='o', label=f"Right Wheel: y={m_r:.3f}x+{b_r:.3f}")
    # print slope and intercept
    Kff_r = 1/m_r
    print(f"Kff_r = {Kff_r}")
    print(f"ff_start_r = {-b_r/m_r}")
    
    o_ax.legend(loc="lower right")
    o_fig.tight_layout()
    svg_path = script_dir / f"velocity_vs_pulse.svg"
    o_fig.savefig(svg_path, format="svg", bbox_inches="tight")


if __name__ == "__main__":
    main()
