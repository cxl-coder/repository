function u_k=Prediction(x_k,E,H,N,p)
 U_k = zeros(N*p,1);%����һ��U_k,NP*1
 U_k = quadprog(H,x_k'*E);%��ȡMin J ���U_k��ֵ
 u_k = U_k(1:p,1);%ȡ��һ�����
end
