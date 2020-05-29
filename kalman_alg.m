clear; close all; clc;

%matrix for vars x1,x2
A = [0.8   2;
     0   0.9];

%matrix for w
B = [1  0;
     0  1];

 %matrix for y
C = [1 0; 0 1];

%matrix for U (u1 = 3, u2=5)
U = [3 5];

Ek = [0 0;
      0 0];
 
Pew = [0 0;
       0 0];

Ew1 = [0 0;
       0 0];
   
   %make w as w1 and w2 vec
   
Ew2 = Ew1;
Ee = Ew1;
Ex0 = Ew1;
   
R = 6;
 
%P(0) 
Pkk = [40 0;
        0 40];
    
Q = [10 0;
     0 10];

% Gammak = B;
% Phi = A;
% Gpk=Gammak *Pew * R.^(-1); % (27)
% Ck = C;
% Fk = U;

Pinit = Pkk;
Xinit = [0;
         0];
u = U;
t=[1:2];
kalmanFilter(A,B,C,Q,R,Pinit,Xinit,u,t)


% Pk1k = (Phi - Gpk*Ck) *Pkk * (Phi - Gpk*Ck)' + Gammak*Q*Gammak' - Gpk*R*Gpk'; % (26)
% 
% Gk1 = Pk1k *Ck' * ( Pk1k * Ck' + R).^(-1) ; % (25)
% 
% Pk1k1 = (1-Gk1*Ck)*Pk1k; % (24) %πίνακας συμμεταβλητότητας X(k+1/k+1)
%                                         %της ανέλιξης του σφάλματος


% Xhatk1k1 = Phi * Xhatkk + Fk*Uk + Gammak*Ew1 + Gk1 * ( yk1 - Ek*Uk - Ee ...
% -Ck * (Phi * Xhatkk + Fk*Uk + Gammak*Ew1          % (23)

% Xk1= Phi * Xk + Fk*Uk + Gammak*Wk; % (14)
% Yk1 = Ck*Xk + Ek*Uk +ek; % (15)

% for i = 1:k
%     Pk1k = A*S.P{i-1}*A'+B*Q*B';
% end
% Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
% 
% Q = 10; 
% R = 6;
% [kalmf,L,P,M] = kalman(Plant,Q,R);
% 
% kalmf = kalmf(1,:);
% 
% a = A;
% b = [B B 0*B];
% c = [C;C];
% d = [0 0 0;0 0 1];
% P = ss(a,b,c,d,-1,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'});
% 
% sys = parallel(P,kalmf,1,1,[],[]);
% 
% SimModel = feedback(sys,1,4,2,1);   % Close loop around input #4 and output #2
% SimModel = SimModel([1 3],[1 2 3]); % Delete yv from I/O list
% 
% t = [0:100]';
% u = sin(t/5);
% 
% n = length(t);
% rng default
% w = sqrt(Q)*randn(n,1);
% v = sqrt(R)*randn(n,1);
% 
% [out,x] = lsim(SimModel,[w,v,u]);
% 
% y = out(:,1);   % true response
% ye = out(:,2);  % filtered response
% yv = y + v;     % measured response
% 
% 
% subplot(211), plot(t,y,'--',t,ye,'-'), 
% xlabel('No. of samples'), ylabel('Output')
% title('Kalman filter response')
% subplot(212), plot(t,y-yv,'-.',t,y-ye,'-'),
% xlabel('No. of samples'), ylabel('Error')