import sympy as sy

A = sy.Matrix([[sy.Symbol("t0")**3, sy.Symbol("t0")**2, sy.Symbol("t0"), 1], [3 * sy.Symbol("t0")**2, 2 * sy.Symbol("t0"), 1, 0], 
               [sy.Symbol("tf")**3, sy.Symbol("tf")**2, sy.Symbol("tf"), 1], [3 * sy.Symbol("tf")**2, 2 * sy.Symbol("tf"), 1, 0]])

x = sy.Matrix([sy.Symbol("a3"), sy.Symbol("a2"), sy.Symbol("a1"), sy.Symbol("a0")])

b = sy.Matrix([sy.Symbol("thetaS"), 0, sy.Symbol("thetaF"), 0])

ans = A.inv() * b

sy.pprint(sy.simplify(ans.subs({sy.Symbol("t0"):0})))

