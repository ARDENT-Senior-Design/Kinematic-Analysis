% function q = Leg_Inverse_Kinematics(p)
p = [0.1; 0.1;0.1]
q0 = p'
ardent_leg = Leg_Info()
TF = zeros(4)
TF(1:3,4) = p
m = [1 1 1 0 0 0]
q = ardent_leg.ikine(TF, 'mask', m)
