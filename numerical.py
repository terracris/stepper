from scipy.optimize import fsolve
import numpy as np

# Define the equation
def equation(delta):
    return 0.577 * delta - 2.4 * np.exp(-400 * delta)

# Initial guess for delta
initial_guess = 0.001

# Solve the equation
delta_solution = fsolve(equation, initial_guess)[0]

print("Delta solution:", round(delta_solution, 7))

