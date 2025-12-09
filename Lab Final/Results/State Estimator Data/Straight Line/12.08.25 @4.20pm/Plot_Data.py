from pathlib import Path
import csv
import matplotlib.pyplot as plt


def plot_pairs(file_path, svg_folder, pairs, delineator):
    """Plot multiple x/y pairs; None means use row index for that axis."""
    with open(file_path, newline="") as f:
        reader = csv.DictReader(f)
        if reader is None or not reader.fieldnames:
            print("No data found")
            return
        cols = {h: [] for h in reader.fieldnames}
        for row in reader:
            for h, v in row.items():
                cols[h].append(v)

    row_indices = list(range(len(next(iter(cols.values()), []))))
    for vs, symb, units, square in pairs:
        x_col, y_col = vs
        x_symb, y_symb = symb
        x_units, y_units = units
        if x_col is None:
            xs_raw = row_indices
        elif x_col in cols:
            xs_raw = cols[x_col]
        else:
            print(f"Skipping pair ({x_col}, {y_col}) - missing x column")
            continue

        if y_col is None:
            ys_raw = row_indices
        elif y_col in cols:
            ys_raw = cols[y_col]
        else:
            print(f"Skipping pair ({x_col}, {y_col}) - missing y column")
            continue

        xs, ys = [], []
        for a, b in zip(xs_raw, ys_raw):
            try:
                xs.append(float(a))
                ys.append(float(b))
            except (TypeError, ValueError):
                # skip rows with bad data
                continue

        if not xs:
            print(f"Skipping pair ({x_col}, {y_col}) - no numeric data")
            continue

        x_label = x_col if x_col is not None else "index"
        y_label = y_col if y_col is not None else "index"

        plt.figure()
        plt.plot(xs, ys, label=f"{y_label} vs {x_label}")
        plt.xlabel(rf"{x_label}, ${x_symb}$, [{x_units}]")
        plt.ylabel(rf"{y_label}, ${y_symb}$, [{y_units}]")
        plt.title(f"{y_label} vs {x_label}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        if square:
            plt.axis("equal")
        plt.savefig(svg_folder / f"({delineator}) {y_label} vs {x_label}.svg", format="svg", bbox_inches="tight")

    # plt.show()


def main():
    svg_folder = Path(__file__).resolve().parent / "plotted_data"
    svg_folder.mkdir(parents=True, exist_ok=True)  # creates it and any missing parents; no error if it exists
    file_name = Path(__file__).resolve().parent / "data.csv"
    pairs = [
        (("x", "y"), ("x", "y"), ("-", "-"), False),
    ]
    plot_pairs(file_name, svg_folder, pairs, "data")


if __name__ == "__main__":
    main()
