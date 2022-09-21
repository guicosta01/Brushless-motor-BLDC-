from configparser import MAX_INTERPOLATION_DEPTH
import sys
from tkinter import Y
import control as clt
import matplotlib.pyplot as plt
from scipy.signal import lti,step

Ti = 0.001
Kp = 0.01404
Ki = Kp/Ti
Kd = 0

#resolucao 
dots = 5000

num = [10.158]
den = [0.1234,1]

G= clt.TransferFunction(num,den)
print('G(s) ',G)

controller = Kp+(1+clt.TransferFunction([Ki],[1,0])+clt.TransferFunction([Kd,0],[1]))
print('Controller = ',controller)

sys = clt.feedback(controller*G)
print("sys= ",sys)

#degrau CL
sys2 = sys.returnScipySignalLTI()[0][0]
t2,y2 = step(sys2,N = dots)

#degrau OP
G2 = G.returnScipySignalLTI()[0][0]
t3,y3 = step(G2,N = dots)


#so pra ver o grafico do step
Tstep = [0,0,1e-15,t2.max()]
Ystep = [0,0,1,1]

Pico = max(y2)
print("Pico= ",Pico)
print('Estabiliza em = ', y2[-1])



#grafico CL
plt.figure(0)
plt.title('Step Response CL')
plt.plot(t2,y2)
plt.plot(Tstep,Ystep)

#grafico OL
plt.figure(1)
plt.title('Step Response OL')
plt.plot(t3,y3)
plt.show()





