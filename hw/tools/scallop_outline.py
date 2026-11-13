import math, json
R=29.8; cv=(0.0,-14.305); A=math.degrees(20.51/29.8)          # 39.434
Rc=R-2.0; cc=(-40.166,29.8)
P=lambda c,r,a:(c[0]+r*math.cos(math.radians(a)), c[1]+r*math.sin(math.radians(a)))
N=8; step=2*A/N
tang=[90+A-k*step for k in range(N+1)]
mid=[P(cv,R,tang[0])]+[P(cv,R/math.cos(math.radians(step/2)),(tang[i]+tang[i+1])/2) for i in range(N)]+[P(cv,R,tang[-1])]
side=[P(cc,Rc,270+A*k/2) for k in range(3)]                   # T0, V1, E (left)
# notch3 / first chord intersections
c3,r3=(-35.0,4.0),3.75
(x0,y0),(x1,y1)=side[0],side[1]; dx,dy=x1-x0,y1-y0; fx,fy=x0-c3[0],y0-c3[1]
a=dx*dx+dy*dy; b=2*(fx*dx+fy*dy); c=fx*fx+fy*fy-r3*r3; D=math.sqrt(b*b-4*a*c)
I=[(x0+t*dx,y0+t*dy) for t in sorted([(-b-D)/(2*a),(-b+D)/(2*a)])]
angI=[math.degrees(math.atan2(p[1]-c3[1],p[0]-c3[0])) for p in I]
left=[side[0],I[0],("ARC",c3,r3,angI[1],angI[0]+360),I[1],side[1],side[2]]
chain=left+mid+[(-x,y) for x,y in reversed(side)]
K=lambda p:[round(150+p[0],6),round(100-p[1],6)]
out=[]
prev=None
for item in chain:
    if isinstance(item,tuple) and item and item[0]=="ARC":
        out.append({"arc":True,"cx":150+item[1][0],"cy":100-item[1][1],"r":item[2],"a0":round(item[3],6),"a1":round(item[4],6)})
        prev=None; continue
    p=K(item)
    if prev is not None: out.append({"x1":prev[0],"y1":prev[1],"x2":p[0],"y2":p[1]})
    prev=p
print(json.dumps(out))
print("recess max",R/math.cos(math.radians(step/2))-R)
