import numpy as np
import matplotlib.pyplot as plt
# from tqdm.notebook import tqdm as tqdm
from tqdm import tqdm

'''
def rk4(func, t, h, y, *x)
ルンゲ・クッタ法を一回分計算する関数
    引数リスト
    func:導関数
    t：現在時刻を表す変数
    h：刻み幅
    y：出力変数（求めたい値）
    *x:引数の数が可変する事に対応する、その他の必要変数
※この関数では時刻は更新されないため、これとは別に時間更新をする必要があります。
'''
def rk4(func, t, h, y, *x):
    k1=h*func(t, y, *x)
    k2=h*func(t+0.5*h, y+0.5*k1, *x)
    k3=h*func(t+0.5*h, y+0.5*k2, *x) 
    k4=h*func(t+h, y+k3, *x)
    y=y+(k1 + 2*k2 + 2*k3 + k4)/6
    return y

'''
導関数の書き方
def func(t, y, *state):
    func:自分で好きな関数名をつけられます
    t:時刻変数(変数の文字はtで無くても良い) 
    y:出力変数(変数の文字はyで無くても良い)
    *state:その他の必要変数(引数の数は可変可能))
#関数サンプル
def vdot(t, y, *state):
    s1=state[0]
    s2=state[1]
    return t+y+s1+s2
    
'''
#以下ロボットの位置と速度を計算するためルンゲクッタソルバに渡す導関数

def uDot(t, u, m, fx ,r, v):
    return fx/m+r*v

def vDot(t, v, m, fy, r, u):
    return fy/m-r*u

def rDot(t, r, Izz, N):
    return N/Izz

def xDot(t, x, u, v, psi):
    return u*np.cos(psi) -v*np.sin(psi)

def yDot(t, y, u, v , psi):
    return u*np.sin(psi) +v*np.cos(psi)

def psiDot(t, psi, r):
    return r

#初期化
U=np.empty(0)
V=np.empty(0)
R=np.empty(0)
X1=np.empty(0)
Y1=np.empty(0)
X2=np.empty(0)
Y2=np.empty(0)
X3=np.empty(0)
Y3=np.empty(0)
X4=np.empty(0)
Y4=np.empty(0)
X5=np.empty(0)
Y5=np.empty(0)
Psi=np.empty(0)
T=np.empty(0)

u=1
v=0
r=0
x=0
y=0
psi=0
t=0
Fx=0
Fy=0
Izz=1
N=0
s=0 #積分器
beta=0

#質量
m=0.1

#
#コーナリングパワー
###################
K=20

#刻み幅
h=1e-5

#制御周期
Tc=1e-3

#旋回パラメータ
psiref=np.pi
omegadot=385 #330
psi24=np.pi/4
TW1=0.01
TW24=np.sqrt(2*psi24/omegadot)
TW3=np.sqrt(omegadot/2/psi24)*(psiref-2*psi24)/omegadot
TIME=int((TW1*2+TW24*2+TW3)/h)

#制御を1msでするためのカウンタ
cnt=0

#求解ループ
for n in tqdm(range(TIME)):
    
    #外力の計算（制御）
    Fy=-K*beta
    if t<TW1:
        N=0.0
        X1=np.append(X1, x)
        Y1=np.append(Y1, y)
    elif t<TW1+TW24:
        N=omegadot
        X2=np.append(X2, x)
        Y2=np.append(Y2, y)
    elif t<TW1+TW24+TW3:
        N=0
        X3=np.append(X3, x)
        Y3=np.append(Y3, y)
    elif t<TW1+TW24*2+TW3:
        N=-omegadot
        X4=np.append(X4, x)
        Y4=np.append(Y4, y)
    else:
        N=0.0
        X5=np.append(X5, x)
        Y5=np.append(Y5, y)
        
    #データのサンプリングと制御
    if cnt==int(Tc/h):
        cnt=0
        err=1-np.sqrt(u**2+v**2)
        s=s+err
        Fx=K*beta**2+25.0*err+0.06*s
        #Fx=100.0*err+0.01*s
        #Fx=0 #制御西にするときはコメントをはずす
    cnt=cnt+1
    
    
    U=np.append(U, u)
    V=np.append(V, v)
    R=np.append(R, r)
    Psi=np.append(Psi, psi)
    T=np.append(T, t)
    uold=u
    vold=v
    rold=r
    xold=x
    yold=y
    psiold=psi
    
    #数値積分（ルンゲクッタ関数呼び出し）
    u=rk4(uDot, t, h, uold, m, Fx, rold, vold)
    v=rk4(vDot, t, h, vold, m, Fy, rold, uold)
    r=rk4(rDot, t, h, rold, Izz, N)
    x=rk4(xDot, t, h, xold, uold, vold, psiold)
    y=rk4(yDot, t, h, yold, uold, vold, psiold)
    psi=rk4(psiDot, t, h, psiold, rold)
    
    t=t+h
    beta=np.arctan2(v, u)



#odometry test

dt=1e-3
eps=1e-8
smpling=int(dt/h)

u=U[0::smpling]
v=V[0::smpling]
psi=Psi[0::smpling]
r=R[0::smpling]

x=0
y=0
beta=0


Xo=np.array([0])
Yo=np.array([0])
Beta=np.array([0])
Psio=np.array([0])

#オドメトリの選択
#swの値で計算を切り替える
#0:直線近似
#1:円弧近似
#2:横滑り考慮した直線近似
#3:横滑り考慮した円弧近似
sw=3

firstflag=1
for us,vs,psis,rs in zip(u, v, psi, r):
    

    if firstflag==1:
        firstflag=0
        dus=us
        dvs=vs
        dpsis=psis
        drs=rs
        dbeta=beta
        continue
        
    #オドメトリの計算方法で場合分け
    #直線近似
    if sw==0:
        dVs=np.sqrt(dus**2+dvs**2)
        x=x+dt*dVs*np.cos(dpsis)
        y=y+dt*dVs*np.sin(dpsis)
    #円弧近似
    elif sw==1:
        deltapsi=psis-dpsis
        dVs=np.sqrt(dus**2+dvs**2)
        if np.sqrt(drs**2)<eps:
            x=x+dt*dVs*np.cos(dpsis)
            y=y+dt*dVs*np.sin(dpsis)
        else:
            x=x+2*dVs/drs*np.cos(dpsis+ deltapsi/2)*np.sin(deltapsi/2)
            y=y+2*dVs/drs*np.sin(dpsis+ deltapsi/2)*np.sin(deltapsi/2)

    #横滑り考慮した直線近似
    elif sw==2:
        dVs=np.sqrt(dus**2+dvs**2)
        x=x+dt*dVs*np.cos(dpsis + dbeta)
        y=y+dt*dVs*np.sin(dpsis + dbeta)
        beta=dbeta-dt*(K*dbeta/m/dVs + drs)
    #横滑り考慮した円弧近似
    elif sw==3:
        dVs=np.sqrt(dus**2+dvs**2)
        beta=dbeta-dt*(K*dbeta/m/dVs + drs)
        deltapsi=(psis+beta)-(dpsis+dbeta)
        omega=deltapsi/dt
        if np.sqrt(omega**2)<eps:
            x=x+dt*dVs*np.cos(dpsis+dbeta)
            y=y+dt*dVs*np.sin(dpsis+dbeta)
        else:
            x=x+2*dVs/omega*np.cos(dpsis+dbeta+ deltapsi/2)*np.sin(deltapsi/2)
            y=y+2*dVs/omega*np.sin(dpsis+dbeta+ deltapsi/2)*np.sin(deltapsi/2)

    dus=us
    dvs=vs
    dpsis=psis
    drs=rs
    dbeta=beta

    
    
    Xo=np.append(Xo, x)
    Yo=np.append(Yo, y)
    Beta=np.append(Beta, beta)

plt.figure(figsize=(10,10))

plt.xlim(0, 0.11)
plt.ylim(0, 0.11)
plt.xticks( np.arange(0.0, 0.11, 0.01) )
plt.yticks( np.arange(0.0, 0.11, 0.01) )
plt.grid()




plt.plot(X1,Y1, lw=5)
plt.plot(X2,Y2, lw=5)
plt.plot(X3,Y3, lw=5)
plt.plot(X4,Y4, lw=5)
plt.plot(X5,Y5, lw=5)
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.plot(Xo, Yo, '.-', c='red')
plt.show()