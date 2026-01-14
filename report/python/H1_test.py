import numpy as np
from scipy.stats import mannwhitneyu

POLLS = 22

# frequencies: (Very Poor=1, Poor=2, Acceptable=3, Good=4, Very Good=5)
ear = [int(round(x * POLLS / 100.0)) for x in [5, 18, 41, 27, 19]]  # Sad
ear += [int(round(x * POLLS / 100.0)) for x in [18, 32, 14, 27, 9]]  # Happy
no_ear = [int(round(x * POLLS / 100.0)) for x in [18, 36, 27, 14, 5]]  # Interested
no_ear += [int(round(x * POLLS / 100.0)) for x in [27, 23, 23, 23, 5]]  # Annoyed



print("ear group frequencies:", ear)
print("no_ear group frequencies:", no_ear)

print("\n Sum for ear group:", sum(ear))
print(" Sum for no_ear group:", sum(no_ear))

# Expand to raw scores
ear_scores = []
no_ear_scores = []
for i, freq in enumerate(ear):
    ear_scores += [i+1] * freq
for i, freq in enumerate(no_ear):
    no_ear_scores += [i+1] * freq

U_stat, p = mannwhitneyu(ear_scores, no_ear_scores, alternative='greater')
print(f"U = {U_stat}, p = {p}")