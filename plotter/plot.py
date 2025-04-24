import pandas as pd
import matplotlib.pyplot as plt
import sys

# Read the CSV file, using the first row as column names
df = pd.read_csv(sys.argv[1])

# Display the first few rows of the dataframe
print(df.head())

# Dynamically get the column names
columns = df.columns.tolist()

# Check if there are at least two columns to plot
if len(columns) < 2:
    raise ValueError("The CSV file must have at least two columns to plot.")

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

# Show the plot
plt.tight_layout()
plt.show()
