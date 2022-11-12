function u_k=Prediction(x_k,E,H,N,p)
 U_k = zeros(N*p,1);%定义一个U_k,NP*1
 U_k = quadprog(H,x_k'*E);%求取Min J 后的U_k的值
 u_k = U_k(1:p,1);%取第一个结果
end
