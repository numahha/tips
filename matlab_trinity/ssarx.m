clear all;

downsample=5;

y = readmatrix(append('processeddata',num2str(downsample),'\\y1.csv'));
u = readmatrix(append('processeddata',num2str(downsample),'\\u1.csv'));
d = iddata(y,u,0.2*downsample);
for i = 2:82
    y = readmatrix(append('processeddata',num2str(downsample),'\\y',num2str(i),'.csv'));
    u = readmatrix(append('processeddata',num2str(downsample),'\\u',num2str(i),'.csv'));
    temp_d = iddata(y,u,0.2*downsample);
    d = merge(d,temp_d);
end

nx = 1; % ssarx

%sys = n4sid(d,1:10);
sys = n4sid(d,nx,n4sidOptions('N4Weight','SSARX'));
N=20;
t = 0.2*downsample*(0:N-1);
ux = [1 zeros(1,N-1)];
[b1,a1] = ss2tf(sys.A,sys.B,sys.C,sys.D,1);
y1u1 = filter(b1(1,:),a1,ux);
[b2,a2] = ss2tf(sys.A,sys.B,sys.C,sys.D,2);
y1u2 = filter(b2(1,:),a2,ux);
[b3,a3] = ss2tf(sys.A,sys.B,sys.C,sys.D,3);
y1u3 = filter(b3(1,:),a3,ux);
[b4,a4] = ss2tf(sys.A,sys.B,sys.C,sys.D,4);
y1u4 = filter(b4(1,:),a4,ux);
%[b5,a5] = ss2tf(sys.A,sys.B,sys.C,sys.D,5);
%y1u5 = filter(b5(1,:),a5,ux);
%[b6,a6] = ss2tf(sys.A,sys.B,sys.C,sys.D,6);
%y1u6 = filter(b6(1,:),a6,ux);

stem(t,[y1u1;y1u2;y1u3;y1u4]','.');
legend('a_1','a_2','a_3','a_4');
%sys.A
%sys.B
%sys.C
%sys.D