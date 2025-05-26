import os
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

def plot_and_save(csv_file, folder_path):
    """
    Reads a CSV file, plots each Y column against Time/X column in separate plots,
    and saves them as images in the same folder.
    """
    df = pd.read_csv(csv_file)
    columns = df.columns.tolist()

    if len(columns) < 2:
        print(f"Skipping {csv_file}: Not enough columns to plot.")
        return

    x_column = columns[0]
    x_data = df[x_column]

    for y_column in columns[1:]:
        plt.figure(figsize=(10, 6))
        plt.plot(x_data, df[y_column], marker='', linestyle='-', label=y_column)

        plt.xlabel(x_column)
        plt.ylabel(y_column)
        plt.title(f'{y_column} over {x_column}')
        plt.grid(True)
        plt.legend()

        start = int(np.floor(min(x_data)))
        end = int(np.ceil(max(x_data)))
        plt.xticks(x_data)
        labels = [str(val) if i % 10 == 0 else '' for i, val in enumerate(x_data)]
        plt.gca().set_xticklabels(labels, rotation=60, ha='right')
        plt.xlim(start, end)
        plt.tight_layout()

        # Save in the same folder as the CSV
        plot_filename = f"{os.path.splitext(os.path.basename(csv_file))[0]}_{y_column}_plot.png"
        plot_path = os.path.join(folder_path, plot_filename)
        plt.savefig(plot_path, bbox_inches='tight')
        plt.close()
        print(f"Plot saved: {plot_path}")

def main(root_folder):
    """
    Recursively processes all CSV files in the folder and subfolders.
    """
    if not os.path.isdir(root_folder):
        print(f"Error: Folder '{root_folder}' does not exist.")
        return

    for dirpath, _, filenames in os.walk(root_folder):
        for filename in filenames:
            if filename.endswith('.csv'):
                csv_file = os.path.join(dirpath, filename)
                print(f"Processing: {csv_file}")
                plot_and_save(csv_file, dirpath)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plotv3.py <folder_path>")
    else:
        folder_path = sys.argv[1]
        main(folder_path)
