# sensorbasedrobotics

## How to use this:
1. Install V-REP Simulator
2. Install Jupyter Notebook
3. Run the v-rep with corresponding model and import the code

## Algorithm explanation:
5th order Polynomial Trajectory Generation:
Taking the 5th order polynomial:
s(t) = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
v(t) = a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
a(t) = 2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
Substituting the condition: 
s(0)=v(0)=a(0)=v(t)=a(t)=0 and s(t) = xe
It can be represented as A * x = b
Where A = [[1, 0, 0, 0,0,0],
                [1, T, T**2, T**3,T**4,T**5],
                [0, 1, 0, 0, 0, 0],
                [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                [0,0,2,0,0,0],
                [0,0,2,6*T,12*T**2,20*T**3]])  
X = [a0,a1,a2,a3,a4,a5] and b = [0, desiredgoal, 0, 0, 0, 0]
X then can be calculated by A(inverse) * b for an invertible A.

Notice that not all A matrix are invertible. Hence, this is where the next loop comes in:
for T in np.arange(Min_T, Max_T):
            xqp = get_coef_(sx , sxv, sxa, gx, gxv, gxa, T)
            yqp = get_coef_(sy , syv, sya, gy, gyv, gya, T)
            zqp = get_coef_(sz , szv, sza, gz, gzv, gza, T)

This loop of the code is to find, for any given A matrix in the range of T in between Min_T and Max_T.
This loop giving a deadline for the robot to reach the destination goal. This is to prevent singular maxtrix deadlock.

The next part of the code is recalculating the distance, velocity and acceleration for the possible values of coefficient found previously.
for t in np.arange(0.0, T + dt, dt):
                time.append(t)
                rx.append(calc_point(xqp[0],xqp[1],xqp[2],xqp[3],xqp[4],xqp[5],t))
                ry.append(calc_point(yqp[0],yqp[1],yqp[2],yqp[3],yqp[4],yqp[5],t))
                rz.append(calc_point(zqp[0],zqp[1],zqp[2],zqp[3],zqp[4],zqp[5],t))
