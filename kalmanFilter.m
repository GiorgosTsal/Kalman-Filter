function S = kalmanFilter(A,B,C,Q,R,Pinit,Xinit,u,t)
x_len = length(Xinit);

Q=eye(x_len,x_len).*Q; %initial statistical independent variables same will apply to P 
Pkk=eye(x_len,x_len).*Pinit;
 
noiseVec = randn(2, length(t)); 
w = Q*noiseVec;
e = sqrt(R)*randn(1,length(t));
S.w=w;
S.e=e;
S.Q=Q;


S.P{1}=Pkk;

S.len=x_len;
S.x = ones(2,length(t));
S.x(:,1)=Xinit;
yk1= C*Xinit+e(1);
S.y=ones(1,length(t));
S.y(1)=yk1;
S.X=ones(2,length(t));
S.X(:,1)=Xinit;
for i=2:length(t)
    Pk1k = A*S.P{i-1}*A'+B*Q*B';
    Gk1=Pk1k*C'*(C*Pk1k*C'+R)^-1;
    Pk1k1=(ones(2,2)-Gk1*C)*Pk1k;
    S.P{i}=Pk1k1;
    xk1k=A*S.x(:,i-1)+B*u(:,i-1);
    S.X(:,i)=xk1k+B*w(:,i);
    yk1= C*xk1k+e(i);
    S.y(i)=yk1;
    S.x(:,i)=xk1k+B*w(:,i)+Gk1*(yk1-C*xk1k);
end
end
