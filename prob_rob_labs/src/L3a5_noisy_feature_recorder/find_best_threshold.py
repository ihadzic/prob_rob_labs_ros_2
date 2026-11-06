import numpy as np
import matplotlib.pyplot as plt

# Load recorded measurements
open_data = np.atleast_1d(np.loadtxt("open.txt"))
closed_data = np.atleast_1d(np.loadtxt("closed.txt"))

# Generate candidate thresholds
values = np.sort(np.unique(
    np.concatenate([open_data, closed_data])
))

candidates = np.concatenate((
    [np.nextafter(values[0], -np.inf)],
    (values[:-1] + values[1:]) / 2,
    [values[-1]]
))

best_threshold = None
best_error = float('inf')

for threshold in candidates:

    # Door open: values should be <= threshold
    false_negative = np.mean(open_data > threshold)

    # Door closed: values should be > threshold
    false_positive = np.mean(closed_data <= threshold)

    # Equal importance for both errors
    error = (false_negative + false_positive) / 2

    if error < best_error:
        best_error = error
        best_threshold = threshold

print(f"Best threshold: {best_threshold:.4f}")

# Compute four probabilities
p_open_open = np.mean(open_data <= best_threshold)
p_closed_open = np.mean(open_data > best_threshold)
p_open_closed = np.mean(closed_data <= best_threshold)
p_closed_closed = np.mean(closed_data > best_threshold)

print(f"P(z=open | x=open) = {p_open_open:.4f}")
print(f"P(z=closed | x=open) = {p_closed_open:.4f}")
print(f"P(z=open | x=closed) = {p_open_closed:.4f}")
print(f"P(z=closed | x=closed) = {p_closed_closed:.4f}")

print(f"Balanced error rate = {best_error:.2%}")

# Plot original sample positions
open_pred = open_data <= best_threshold
closed_pred = closed_data <= best_threshold

plt.figure(figsize=(12, 6))

plt.scatter(np.where(open_pred)[0], open_data[open_pred],
            color='green', s=10, label='z=open, x=open')

plt.scatter(np.where(~open_pred)[0], open_data[~open_pred],
            color='red', s=15, label='z=closed, x=open')

plt.scatter(np.where(closed_pred)[0], closed_data[closed_pred],
            color='orange', s=15, label='z=open, x=closed')

plt.scatter(np.where(~closed_pred)[0], closed_data[~closed_pred],
            color='blue', s=10, label='z=closed, x=closed')

plt.axhline(best_threshold, color='black', linestyle='--',
            label=f'Threshold = {best_threshold:.4f}')

plt.xlabel('Sample Index')
plt.ylabel('Feature Mean')
plt.title('Door State Classification')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('door_classification.png', dpi=300)
plt.show()
