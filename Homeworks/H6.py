import sympy as sy

theta = sy.Symbol("theta")
Ixx = sy.symbols("Ixx")
Iyy = sy.symbols("Iyy")
Izz = sy.symbols("Izz")

R01 = sy.Matrix([[sy.cos(theta), -sy.sin(theta), 0], [sy.sin(theta), sy.cos(theta), 0], [0, 0, 1]])
R02 = R01
I = sy.Matrix([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])

sy.pprint(sy.simplify(R01 * I * R01.T))