import numpy as np

def calculate_gamma(n):
    return sum(1 / k for k in range(1, n + 1)) - np.log(n)

# Set n to a large value for better accuracy
n = 1000000
gamma_approx = calculate_gamma(n)

print("Approximation of Euler-Mascheroni constant:", gamma_approx)

