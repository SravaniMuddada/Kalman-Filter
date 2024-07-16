%Three state estimation using a Kalman filter
clc;
clear all;
T_p(1)=1;
T_v(1)=5;
T_a(1)=0;
m_p(1)=0;
T=0.5;
t=0:T:1000;
H=[1 0 0; 0 1 0];
for k=2:1:length(t)
    T_p(k)=T_p(k-1)+T_v(k-1)*T+(1/2)*T_a(k-1)*T^2;
    m_p(k)=T_p(k)+1000*randn(1);
    T_v(k)=T_v(k-1)+T_a(k-1)*T;
    m_v(k)=T_v(k)+100*randn(1);
    T_a(k)=randn(1)/10;
end
A=[1 T^2 T^2/2 ; 0 1 T ; 0 0 1]; 
Q = [T^2/20 T^2/8 T^2/6 ; T^3/8 T^2/3 T/2 ; T^2/6 T/2 1];
R = [50 0 ; 0 80];P = zeros(size(Q));
Xe=[0 0 0]';
for k=1:length(T_p)
    if(k==1)
    x1 = A*Xe ;
    P1 = A*P*A' + Q;
    else
    x1 = A*Xe(:,k-1);
    P1 = A*P*A' + Q;
    end
    K = P1*H'*inv(H*P1*H' + R);
    Xe(:,k) = x1 + K*([m_p(k);m_v(k)] - H*x1);
    P = (eye(size(P)) - K*H)*P1;
end
figure,
subplot(311),u =plot(t(1:10:end),T_p(1:10:end),'g','LineWidth',1);hold on
subplot(311),v =plot(t(1:10:end),m_p(1:10:end),'r','LineWidth',1);hold on
subplot(311),w =plot(t(1:10:end),Xe(1,(1:10:end)),'b','LineWidth',1);
legend([u,v,w],'True position','Noisy position','Estimated position')
subplot(312),u1 =plot(t(1:10:end),T_v(1:10:end),'g','LineWidth',1);hold on
subplot(312),v1 =plot(t(1:10:end),m_v(1:10:end),'r','LineWidth',1);hold on
subplot(312),w1 =plot(t(1:10:end),Xe(2,(1:10:end)),'b','LineWidth',1);
legend([u1,v1,w1],'True position','Noisy position','Estimated position')
subplot(313),u2 =plot(t(1:10:end),T_a(1:10:end),'g','LineWidth',1);hold on
subplot(313),v2 =plot(t(1:10:end),Xe(3,(1:10:end)),'b','LineWidth',1);
legend([u2,v2],'True position','Estimated position')
