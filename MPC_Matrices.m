function [E,H]=MPC_Matrices(A,B,Q,R,F,N)

n=size(A,1); %A��n*n���󣬵õ�n

p=size(B,2); %B��n*p���󣬵õ�p

M=[eye(n);zeros(N*n,n)];%��ʼ��M����.M������(N+1)n*n�ģ�
                        %��������n*n����I������һ���Ȱ��°벿��д��0
C=zeros((N+1)*n,N*p);%��ʼ��C������һ��������(N+1)n*Np��0

%����M��C
tmp=eye(n); %����һ��n*n��I����
%����M��C
for i=1:N 
    rows=i*n+(1:n);%���嵱ǰ��������i*n��ʼ����n��
    C(rows,:)=[tmp*B,C(rows-n,1:end-p)];%��C��������
    tmp=A*tmp;%ÿһ�ν�tmp���һ��A
    M(rows,:)=tmp;%��M����д��
end

%����Q_bar��R_bar
Q_bar=kron(eye(N),Q);
Q_bar=blkdiag(Q_bar,F);
R_bar=kron(eye(N),R);

%����G,E,H
G=M'*Q_bar*M;
E=M'*Q_bar*C;
H=C'*Q_bar*C+R_bar;
end

