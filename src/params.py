#!/usr/bin/env python3

class params:
    # Constriction coefficients
    kappa = 1
    phi1 = 2.05
    phi2 = 2.05
    phi = phi1 + phi2
    chi = 1.37              # 2*kappa/abs(2-phi-m.sqrt(phi**2-4*phi))

    # other params
    max_iter = 100          # maximum number of iterations
    nPart = 10              # no of drones/particles
    w = chi                 # inertia coefficient
    wdamp = 0.7             # damping coefficent for inertia
    c1 = chi*phi1           # personal acceleration coefficient
    c2 = chi*phi2           # social acceleration coefficient
    showIterinfo = True     # Flag for showing iteration information