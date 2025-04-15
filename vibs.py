import sympy as sy

# Define Symbols
L = sy.Symbol("L")
J = sy.Symbol("J")
Cr = sy.Symbol("Cr")
Cf = sy.Symbol("Cf")
Ct = sy.Symbol("Ct")
Kr = sy.Symbol("Kr")
Kf = sy.Symbol("Kf")
Kt = sy.Symbol("Kt")
Mb = sy.Symbol("Mb")
Mf = sy.Symbol("Mf")
Mr = sy.Symbol("Mr")

# Define Functions
Yb = sy.Symbol("Yb")
Yr = sy.Symbol("Yr")
Yf = sy.Symbol("Yf")
YbDot = sy.Symbol("YbDot")
YrDot = sy.Symbol("YrDot")
YfDot = sy.Symbol("YfDot")
Theta = sy.Symbol("Theta")
ThetaDotDot = sy.Symbol("ThetaDotDot")
ThetaDot = sy.Symbol("ThetaDot")

# Define energy
Kinetic = (1/2 * Mf * YfDot**2) + (1/2 * Mr * YrDot**2) + (1/2 * Mb * YbDot**2) + (1/2 * J * ThetaDot**2)

Dissipative = (1/2 * Ct * YrDot**2) + (1/2 * Ct * YfDot**2) + (1/2 * Cr * (YbDot - L/2 * ThetaDot * sy.cos(Theta) - YrDot)**2) + (1/2 * Cf * (YbDot + L/2 * ThetaDot * sy.cos(Theta) - YfDot)**2)

Potential = (1/2 * Kt * Yr**2) + (1/2 * Kt * Yf**2) + (1/2 * Kr * (Yb - L/2 * sy.sin(Theta) - Yr)**2) + (1/2 * Kf * (Yb + L/2 *  sy.sin(Theta) - Yf)**2)

# Derive equations
eq1 = sy.diff(Kinetic, YrDot) - sy.diff(Kinetic, Yr) + sy.diff(Dissipative, YrDot) + sy.diff(Potential, Yr)
eq2 = sy.diff(Kinetic, YfDot) - sy.diff(Kinetic, Yf) + sy.diff(Dissipative, YfDot) + sy.diff(Potential, Yf)
eq3 = sy.diff(Kinetic, YbDot) - sy.diff(Kinetic, Yb) + sy.diff(Dissipative, YbDot) + sy.diff(Potential, Yb)
eq4 = sy.diff(Kinetic, ThetaDot) - sy.diff(Kinetic, Theta) + sy.diff(Dissipative, ThetaDot) + sy.diff(Potential, Theta)

#sy.pprint(eq1) #good
#sy.pprint(eq2) #good 
#sy.pprint(eq3) #good
#sy.pprint(eq4) #good