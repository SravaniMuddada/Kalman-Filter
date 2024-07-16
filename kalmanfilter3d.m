% 3-D position estimation using a Kalman filter
clc;
clear all;
T_p=[0 0 0]';
T_v=[0 0 0]';
T_a=[0 0 0]';
m_p=[0 0 0]';
T=0.2;
t=0:T:1000;
H=[1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0];
for k=2:1:length(t)
    T_p(:,k)=T_p(:,k-1)+T_v(:,k-1)*T+(1/2)*T_a(:,k-1)*T^2;
    m_p(:,k)=T_p(:,k)+10*randn(3,1);
    T_v(:,k)=T_v(:,k-1)+T_a(:,k-1)*T;
    m_v(:,k)=T_v(:,k)+10*randn(3,1);
    T_a(:,k)=randn(3,1)/10;
end
A=[1 0 0 T 0 0;0 1 0 0 T 0;0 0 1 0 0 T;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
B=[(T^2)/2 0 0; 0 T^2/2 0;0 0 T^2/2;T 0 0;0 T 0;0 0 T];
Q = [0.0005 0 0 0 0 0;0 0.0005 0 0 0 0;0 0 0.005 0 0 0;0 0 0 0.002 0 0;0 0 0 0 0.002 0;0 0 0 0 0 0.002];
R = 30.*eye(3);
P = zeros(size(Q));
Xe=[0 0 0 0 0 0]';
for k=1:length(T_p)
    if(k==1)
    x1 = A*Xe + B*randn(3,1)/100;
    P1 = A*P*A' + Q;
    else
    x1 = A*Xe(:,k-1) + B*randn(3,1)/100;
    P1 = A*P*A' + Q;
    end

    K = P1*H'*inv(H*P1*H' + R);
    Xe(:,k) = x1 + K*(m_p(:,k) - H*x1);
    P = (eye(size(P)) - K*H)*P1;

end
figure,
subplot(211),u =plot3(T_p(1,(1:10:end)),T_p(2,(1:10:end)),T_p(3,(1:10:end)),'g','LineWidth',1);hold on
subplot(211),v =plot3(m_p(1,(1:10:end)),m_p(2,(1:10:end)),m_p(3,(1:10:end)),'r','LineWidth',1);hold on
subplot(211),w =plot3(Xe(1,(1:10:end)),Xe(2,(1:10:end)),Xe(3,(1:10:end)),'b','LineWidth',1);
legend([u,v,w],'True 3-D position','Noisy 3-D position','Estimated 3-D position')

