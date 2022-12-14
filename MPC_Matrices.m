function [E,H]=MPC_Matrices(A,B,Q,R,F,N)

n=size(A,1); %A是n*n矩阵，得到n

p=size(B,2); %B是n*p矩阵，得到p

M=[eye(n);zeros(N*n,n)];%初始化M矩阵.M矩阵是(N+1)n*n的，
                        %它上面是n*n个“I”，这一步先把下半部分写成0
C=zeros((N+1)*n,N*p);%初始化C矩阵，这一步令它有(N+1)n*Np个0

%定义M和C
tmp=eye(n); %定义一个n*n的I矩阵
%更新M和C
for i=1:N 
    rows=i*n+(1:n);%定义当前行数，从i*n开始，共n行
    C(rows,:)=[tmp*B,C(rows-n,1:end-p)];%将C矩阵填满
    tmp=A*tmp;%每一次将tmp左乘一次A
    M(rows,:)=tmp;%将M矩阵写满
end

%定义Q_bar和R_bar
Q_bar=kron(eye(N),Q);
Q_bar=blkdiag(Q_bar,F);
R_bar=kron(eye(N),R);

%计算G,E,H
G=M'*Q_bar*M;
E=M'*Q_bar*C;
H=C'*Q_bar*C+R_bar;
end

