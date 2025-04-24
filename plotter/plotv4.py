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
    # Read the CSV file
    df = pd.read_csv(csv_file)

    # Get the column names
    columns = df.columns.tolist()

    # Check if there are at least two columns to plot
    if len(columns) < 2:
        print(f"Skipping {csv_file}: Not enough columns to plot.")
        return

    # Time/X column (assumed to be the first column)
    x_column = columns[0]
    x_data = df[x_column]

    # Plot each Y column (all columns except the first) in separate figures
    for y_column in columns[1:]:
        plt.figure(figsize=(10, 6))
        plt.plot(x_data, df[y_column], marker='', linestyle='-', label=y_column)

        # Add labels and title
        plt.xlabel(x_column)
        plt.ylabel(y_column)
        plt.title(f'{y_column} over {x_column}')
        plt.grid(True)
        plt.legend()

        # Rotate x-axis labels if needed
        start = int(np.floor(min(x_data)))
        end = int(np.ceil(max(x_data)))
        plt.xticks(x_data)

        labels = [str(val) if i % 10 == 0 else '' for i, val in enumerate(x_data)]
        plt.gca().set_xticklabels(labels,rotation=60 , ha='right')
        plt.xlim(start, end)
        plt.tight_layout()

        # Save the plot as an image
        plot_filename = f"{os.path.splitext(os.path.basename(csv_file))[0]}_{y_column}_plot.png"
        plot_path = os.path.join(folder_path, plot_filename)
        plt.savefig(plot_path, bbox_inches='tight')
        plt.close()  # Close the plot to free up memory

        print(f"Plot saved: {plot_path}")

def main(folder_path):
    """
    Processes all CSV files in the given folder.
    """
    # Check if the folder exists
    if not os.path.isdir(folder_path):
        print(f"Error: Folder '{folder_path}' does not exist.")
        return

    # Iterate over all CSV files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.csv'):
            csv_file = os.path.join(folder_path, filename)
            print(f"Processing: {csv_file}")
            plot_and_save(csv_file, folder_path)

if __name__ == "__main__":
    # Check if a folder path is provided as a command-line argument
    if len(sys.argv) != 2:
        print("Usage: python plotv3.py <folder_path>")
    else:
        folder_path = sys.argv[1]
        main(folder_path)
