import sympy as sy

Eq = sy.Matrix([[3 * sy.Symbol("m") * sy.Symbol("s")**2 + 5 * sy.Symbol("k"), -2 * sy.Symbol("k"), 0], 
                 [-2 * sy.Symbol("k"), 2 * sy.Symbol("m") * sy.Symbol("s")**2 + 3 * sy.Symbol("k"), -sy.Symbol("k")], 
                 [0, -sy.Symbol("k"), sy.Symbol("m") * sy.Symbol("s")**2 + sy.Symbol("k")]])

deter = Eq.det()

sy.pprint(deter)

freq = sy.solve(deter, sy.Symbol("s"))

print(sy.simplify(freq))