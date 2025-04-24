import pandas as pd
import matplotlib.pyplot as plt
import sys
# Read the CSV file
df = pd.read_csv(sys.argv[1])

# Display the first few rows of the dataframe
print(df.head())

# Plot the data
plt.figure(figsize=(10, 6))

# Plot FreeSpaceLoss vs Distance
plt.plot(df['Distance'], df['FreeSpaceLoss'], marker='o', linestyle='-', color='b', label='FreeSpaceLoss')

# Plot LogDistanceLoss vs Distance
plt.plot(df['Distance'], df['LogDistanceLoss'], marker='s', linestyle='--', color='r', label='LogDistanceLoss')

# Add labels and title
plt.xlabel('Distance')
plt.ylabel('Path Loss (dB)')
plt.title('FreeSpaceLoss and LogDistanceLoss vs Distance')
plt.grid(True)

# Add a legend
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()
