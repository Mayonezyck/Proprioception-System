import numpy as np
import matplotlib.pyplot as plt

# Set random seed for reproducibility
np.random.seed(45)

# Multi-Armed Bandit setup
K = 5  # number of arms
true_probs = np.array([0.45, 0.48, 0.6, 0.47, 0.44])  # true payout probabilities
epsilon = 0.1  # exploration rate
N_pulls = 1000  # total number of pulls/steps

# Initialize estimates and counts
Q = np.zeros(K)          # estimated value for each arm
counts = np.zeros(K)     # times each arm has been pulled

# Track history
Q_history = np.zeros((N_pulls, K))
cumulative_rewards = np.zeros(N_pulls)
arm_selections = np.zeros(N_pulls, dtype=int)

# Run epsilon-greedy
cumulative_reward = 0
for t in range(N_pulls):
    # Epsilon-greedy action selection
    if np.random.rand() < epsilon:
        action = np.random.randint(K)
    else:
        action = np.argmax(Q)
    
    # Pull selected arm
    reward = 1 if np.random.rand() < true_probs[action] else 0
    
    # Update counts and Q
    counts[action] += 1
    Q[action] += (reward - Q[action]) / counts[action]
    
    # Record history
    Q_history[t] = Q.copy()
    cumulative_reward += reward
    cumulative_rewards[t] = cumulative_reward
    arm_selections[t] = action

# Plot estimated probabilities over time
plt.figure(figsize=(10, 6))
for i in range(K):
    plt.plot(Q_history[:, i], label=f"Arm {i+1} (true p={true_probs[i]:.2f})")
plt.xlabel("Pull number")
plt.ylabel("Estimated probability")
plt.title("Estimated Payout Probability of Each Arm Over Time")
plt.legend()
plt.grid(True)
plt.show()

# Plot cumulative rewards 
plt.figure(figsize=(10, 5))
plt.plot(cumulative_rewards, label="Cumulative Reward")
plt.xlabel("Pull number")
plt.ylabel("Total Reward")
plt.title("Cumulative Reward Over Time")
plt.grid(True)
plt.show()

# Plot arm selection counts
final_counts = np.bincount(arm_selections, minlength=K)
plt.figure(figsize=(8, 5))
plt.bar(range(1, K+1), final_counts, tick_label=[f"Arm {i}" for i in range(1, K+1)])
plt.xlabel("Arm")
plt.ylabel("Number of Times Selected")
plt.title("Final Arm Selection Distribution")
plt.show()

# Plot regret over time
best_prob = np.max(true_probs)
optimal_cumulative = best_prob * np.arange(1, N_pulls + 1)
regret = optimal_cumulative - cumulative_rewards

plt.figure(figsize=(10, 5))
plt.plot(regret, label="Regret")
plt.xlabel("Pull number")
plt.ylabel("Regret")
plt.title("Regret Over Time")
plt.grid(True)
plt.show()

