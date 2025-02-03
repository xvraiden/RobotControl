import numpy as np
import sympy as sy
import matplotlib.pyplot as plt


#1
# Makes a list of random values from 0-9, 10 values
randomvalues=np.random.randint(0,9,(10))
# Converts it into numpy array
randomvalues=np.array(randomvalues)
# Prints list
print(f'This is the random values list {randomvalues}')
# Prints mean
print(f'This is the mean of said list: {np.mean(randomvalues)}')

greaterthanmeanvalues=[]
# Checks all values in list and if the value is greater than the mean
for i in randomvalues:
    if i > np.mean(randomvalues):
        # This adds values greater than mean to a new list
        greaterthanmeanvalues.append(i)
# Prints the list of values greater than the mean
print(f'The values that are greater than the mean are: {greaterthanmeanvalues}')

#2

# Makes a list from 0 to 10, 11 steps
problem2list=np.linspace(0,10,11)
# Does a function print
print(f'The list of values from 0 to 10 are: \n{problem2list}')
# Defines x as a sympy symbol
x=sy.Symbol('x')
# Creates a Sympy function
fx=(x)**2
print(f'The function is: {fx}')

listoffunctioncompledvalues=[]

# For the numbers in the list problem2list, go use that function and append the above list
for i in problem2list:
    ans=fx.subs(x,i)
    listoffunctioncompledvalues.append(ans)
# Print it in a nice way
sy.pprint(listoffunctioncompledvalues)

# Plot it, do a color defined
plt.plot(problem2list,listoffunctioncompledvalues,color='#df506a')
# When you got a list you can show a specific one by listname[indexyouwant]
plt.title(f'x**2 plot over the values from {problem2list[0]} to {problem2list[-1]}')
plt.xlabel('X Values')
plt.ylabel('Y Values')
plt.show()
