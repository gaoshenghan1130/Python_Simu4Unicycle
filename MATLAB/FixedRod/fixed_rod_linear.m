function s = fixed_rod_linear(p,c)
% Exact straight-rolling Jacobian and closed-form eigenvalues, BP=0.
assert(p.BP==0,'Closed-form spectrum here assumes BP=0.');
R=p.R; O=c.speed/R;
b=R*p.h*p.mp;
J=p.JPx+p.JR+p.JW2+R^2*(p.mp+p.mw+p.mr)+2*b+p.mp*p.h^2;
G=p.g*(R*(p.mp+p.mw+p.mr)+p.h*p.mp);
C=p.JPz+p.JR+p.JW2;
W=p.JW1+R^2*(p.mp+p.mr+p.mw);
H=W+b;
I=p.JPy+p.mp*p.h^2;
P=p.g*p.h*p.mp;
D=W*I-b^2;
a=p.JW1*O/C;
k=G/J; d=H*O/J;
L=[W b;b I]\[c.kp c.kd;P-c.kp -c.kd];
A=zeros(10);
A(1,6)=k; A(1,3)=d;
A([2 4],[8 4])=L;
A(3,1)=-a;
A(5,3)=1; A(6,1)=1; A(7,2)=1; A(8,4)=1;
A(9,2)=R; A(10,1)=-R; A(10,5)=c.speed;
q=k-d*a;
alpha=(W+b)*c.kd/D;
beta=((W+b)*c.kp-W*P)/D;
lat=[0;sqrt(complex(q));-sqrt(complex(q))];
long=[0;(-alpha+sqrt(complex(alpha^2-4*beta)))/2; ...
    (-alpha-sqrt(complex(alpha^2-4*beta)))/2];
s=struct('A',A,'lateral',lat,'longitudinal',long, ...
    'eigenvalues',[lat;long;zeros(4,1)], ...
    'critical_speed',R*sqrt(G*C/(H*p.JW1)), ...
    'q',q,'a',a,'k',k,'d',d,'J',J,'G',G,'C',C, ...
    'W',W,'H',H,'I',I,'b',b,'Delta',D,'alpha',alpha,'beta',beta);
end
