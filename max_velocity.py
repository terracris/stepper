def calculate_sum(delta):
    n_max = int(1.6 / delta)  # Calculate the maximum value of n
    total = sum(1 / (delta * n) for n in range(1, n_max + 1))
    return total

# Example calculation for delta = 0.01
delta = 0.01
result = calculate_sum(delta)
print("Result:", result)

