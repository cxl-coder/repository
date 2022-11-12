%%����
clear;
close all;
clc;
%%��һ��������״̬�ռ����
%%����״̬����A��n*n����
A=[1 0.1;-1 2];
n=size(A,1);

%%�����������B,n*p����
B=[0.2 1;0.5 2];
p=size(B,2);

%%����Q����n*n����
Q=[100 0;0 1];

%%����F����n*n����
F=[100 0;0 1];

%%����R����p*p����
R=[0.1 0;0 0.1];

%%����step����K
k_steps=100;

%%�������X_K��n*k����
X_K=zeros(n,k_steps);

%%��ʼ��״̬������n*1����
X_K(:,1)=[20;-20];

%%�����������U_K��p*k����
U_K=zeros(p,k_steps);

%%����Ԥ������N
N=5;
%%Call MPC_Matrices���� ���E,H����
[E,H]=MPC_Matrices(A,B,Q,R,F,N);

%%����ÿһ����״̬������ֵ 
for k=1:k_steps
    %%���U_K(:,k)
    U_K(:,k)=Prediction(X_K(:,k),E,H,N,p);
    %�����k+1��ʱ״̬������ֵ
    X_K(:,k+1)=(A*X_K(:,k)+B*U_K(:,k));
end

%%����״̬����������ı仯
subplot(2,1,1);hold;
for i=1:size(X_K,1)
    plot(X_K(i,:));
end
legend("x1","x2")
hold off;

subplot(2,1,2);hold;
for i=1:size(U_K,1)
    plot(U_K(i,:));
end
legend("u1","u2")
