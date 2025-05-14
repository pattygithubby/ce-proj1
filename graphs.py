import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
from sklearn.linear_model import LinearRegression
from matplotlib.ticker import ScalarFormatter

from matplotlib import rcParams

rcParams['mathtext.fontset'] = 'stix'  # Use STIX math font 
rcParams['font.family'] = 'Times New Roman'

# Load data
df = pd.read_excel("arena3/turtlebot3_metrics3.xlsx") #Insert filename that needs graph here
df["cumulative_victims"] = df["avg_victims"].cummax()

# Linear regression (global trend)
X = df["second"].values.reshape(-1, 1)
y = df["avg_speed"].values
model = LinearRegression().fit(X, y)
df["lin_fit"] = model.predict(X)

# Extract slope and intercept
slope = model.coef_[0]
intercept = model.intercept_
equation = rf"$y = {slope:.4f}x + {intercept:.2f}$"

# Plot setup
fig, ax1 = plt.subplots(figsize=(10, 6))

# Raw linear speed 
l1 = ax1.plot(df["second"], df["avg_speed"], color='#0C4A70', label="Average Linear Speed", linewidth=3.5)

# Linear fit 
l2 = ax1.plot(df["second"], df["lin_fit"], color='#3A93C2', linestyle='--', linewidth=3.5, label="Average Linear Speed (fit)")

# Left Y-axis config
ax1.set_xlabel("Time (s)", fontsize=14)
ax1.set_ylabel(r"Linear Speed (m·s$^{-1}$)", fontsize=14, color='black')
ax1.tick_params(axis='y', colors='black', labelsize=12)
ax1.tick_params(axis='x', colors='black', labelsize=12)
ax1.set_ylim(0, 0.25)
ax1.grid(True, linestyle='--', alpha=0.5)
ax1.ticklabel_format(useOffset=False, axis='x')
ax1.xaxis.set_major_formatter(ScalarFormatter(useMathText=False))

# Right Y-axis (Cumulative Victims in Crimson)
ax2 = ax1.twinx()
l3 = ax2.step(df["second"], df["cumulative_victims"], where='post', color='#B10026', linewidth=3.5, label="Victims Detected")
ax2.set_ylabel("Cumulative Victims", fontsize=14, color='black')
ax2.tick_params(axis='y', colors='black', labelsize=12)
ax2.set_ylim(0, df["cumulative_victims"].max() + 1)

# Combine legends
lines = l1 + l2 + list(l3)
labels = [line.get_label() for line in lines]
ax1.legend(lines, labels, loc='upper left', fontsize=12)

# Title and save
plt.title("Average Speed and Cumulative Victims Over Time", fontsize=16, pad=15)
fig.tight_layout()
plt.savefig("graph.png", dpi=300)
plt.show()
