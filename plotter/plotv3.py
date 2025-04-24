import os
import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_and_save(csv_file, folder_path):
    """
    Reads a CSV file, plots its data, and saves the plot as an image in the same folder.
    """
    # Read the CSV file
    df = pd.read_csv(csv_file)

    # Get the column names
    columns = df.columns.tolist()

    # Check if there are at least two columns to plot
    if len(columns) < 2:
        print(f"Skipping {csv_file}: Not enough columns to plot.")
        return

    # Plot the data
    plt.figure(figsize=(10, 6))

    # Plot each column (starting from the second column) against the first column
    for i, column in enumerate(columns[1:]):
        plt.plot(df[columns[0]], df[column], marker='', linestyle='-', label=column)

    # Add labels and title
    plt.xlabel(columns[0])
    plt.ylabel('Values')
    plt.title(f'Multiple Columns over {columns[0]}')
    plt.grid(True)

    # Add a legend
    plt.legend()

    # Rotate x-axis labels for better readability
    plt.xticks(rotation=45)

    # Save the plot as an image
    plot_filename = os.path.splitext(os.path.basename(csv_file))[0] + '_plot.png'
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

    # Iterate over all files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.csv'):
            csv_file = os.path.join(folder_path, filename)
            print(f"Processing: {csv_file}")
            plot_and_save(csv_file, folder_path)

if __name__ == "__main__":
    # Check if a folder path is provided as a command-line argument
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_path>")
    else:
        folder_path = sys.argv[1]
        main(folder_path)
