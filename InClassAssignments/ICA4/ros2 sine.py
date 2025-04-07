import numpy as np

sinSteps = np.empty((100,2))

for i in range(100):
	newStep = np.array([(i+1)*(3)*(1/100), 2*np.sin((i+1)*(3)*(1/100))])
	sinSteps[i] = [newStep[0], newStep[1]]

print(sinSteps)