rng('shuffle')
w = diag([0.1,0.1,2])*rand(3,1);
W = [0 -w(3) w(1); w(3) 0 -w(2); -w(1) w(2) 0];
R = expm(W);

V = logm(R);
w = [0;0;V(2,1)];
W = [0 -w(3) w(1); w(3) 0 -w(2); -w(1) w(2) 0];
R1 = expm(W);
w = [0;0;atan2(R(2,1),R(1,1))];
W = [0 -w(3) w(1); w(3) 0 -w(2); -w(1) w(2) 0];
R2 = expm(W);
norm(logm(R1'*R))
norm(logm(R2'*R))

%%
% rng('shuffle')
rng('default')
w = diag([0.1,0.1,2])*rand(3,1);
W = [0 -w(3) w(1); w(3) 0 -w(2); -w(1) w(2) 0];
R = expm(W);

N = normalize([0.1*rand(1,1);0.1*rand(1,1);1],'norm');
T = rand(3,1)*5;
d = abs(T(3));
H = R+T*N'/d;
HL = H/norm(H);
% HL = H

[V,D] = eig(HL'*HL);
b = D(2,2);
HL = HL/sqrt(b);
V = real(V);
D = real(D/D(2,2));
v1 = V(:,3);
v2 = V(:,2);
v3 = V(:,1);
s1 = D(3,3);
s2 = D(2,2);
s3 = D(1,1);
u1 = (sqrt(1-s3)*v1 + sqrt(s1-1)*v3)/sqrt(s1-s3);
u2 = (sqrt(1-s3)*v1 - sqrt(s1-1)*v3)/sqrt(s1-s3);

U1 = [v2,u1,cross(v2,u1)];
U2 = [v2,u2,cross(v2,u2)];
W1 = [HL*v2, HL*u1, cross(HL*v2,HL*u1)];
W2 = [HL*v2, HL*u2, cross(HL*v2,HL*u2)];
% N
N1 = cross(v2,u1);
N2 = cross(v2,u2);
R;
R1 = W1*U1';
R2 = W2*U2';
% Tn = normalize(T)
T1 = (HL-R1)*N1;
T2 = (HL-R2)*N2;

H1 = R1 +T1*N1';
H2 = R2 + T2*N2';

Hp1 = project(R1,T1,N1)
Hp2 = project(R2,T2,N2)
Hp = project(R,T,N)


function S = ssm(s)

x = s(1);
y = s(2);
z = s(3);

S = [0 -z x; z 0 -y; -x y 0];

end

function H = project(R,T,N)

if (N(3) <0)
    T = T*(-1);
    
end
% T = T/norm(T);
V = logm(R);
V = logm(R);
w = [0;0;V(1,2)];
W = [0 -w(3) w(1); w(3) 0 -w(2); -w(1) w(2) 0];
R = expm(W);
N = [0;0;1];
H = R+T*N';
% H(:,3) = H(:,3);
H = H/norm(H);
D = eig(H'*H);
H = H/sqrt(D(2));

end

