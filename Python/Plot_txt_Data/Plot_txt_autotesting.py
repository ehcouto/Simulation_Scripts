import pandas as pd
import matplotlib.pyplot as plt

# === CONFIGURATION ===
file_path = "osc00012.txt"      # Change this to your file name
delimiter = "\t"            # Change to ',' for CSV, '\t' for tab-separated, etc.

# === LOAD THE FILE ===
try:
    df = pd.read_csv(file_path, delimiter=delimiter, comment='#')

    print("Data loaded successfully.")
    print("Columns:", df.columns.tolist())

    # === PLOT EACH COLUMN ===
    # If the first column is time or x-axis data, use it.
    if df.shape[1] > 1:
        x_col = df.columns[0]
        for col in df.columns[1:]:
            plt.figure()
            plt.plot(df[x_col], df[col], marker='o')
            plt.xlabel(x_col)
            plt.ylabel(col)
            plt.title(f"{col} vs {x_col}")
            plt.grid(True)
        plt.show()
    else:
        df.plot()
        plt.title("Single Column Plot")
        plt.grid(True)
        plt.show()

except Exception as e:
    print(f"Error loading or plotting the file: {e}")
