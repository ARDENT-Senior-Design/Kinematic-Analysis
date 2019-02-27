function T = Jacobian_Gain(state_p)
q = state(1:3,1)
u = state(4:6,1)
ardent_leg = Leg_Info()
T = ardent_leg.jacob0([q(1) q(2) q(3)])'*u