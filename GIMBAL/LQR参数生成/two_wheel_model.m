clc
clear

% 创建一个文件名的变量，初始值为 'text2.xlsx'
filename = 'text8.xlsx';

% 创建一个循环变量，初始值为 1
n = 1;

% 使用 while 循环来检查文件是否存在
while false
    % 检查当前目录下是否有 textn 文件
    if exist(filename,'file') == 2
        % 如果有，就将 n 加一，生成下一个文件名
        n = n + 1;
        filename = ['text',num2str(n),'.xlsx'];
    else

        break;
    end
end

% 模型参数初始化
% 最低位状态：关节电机转角为150°
for i = 1:300
m=0.5;                     %车轮质量，单位为kg
M=7.026;                       %机体质量，单位为kg 11.65 7.026
r=80*10^(-3);              %车轮半径，单位为m
I=1488.358*10^(-6);        %车轮转动惯量，单位为kg*m^2   1488.358
l=(133+i)*10^(-3);             %摆长，单位为m  111.81 150  13.3-43.1 133mm-431mm 300的变化量
Jz=137252.709*10^(-6);      %车体转动惯量，单位为kg*m^2 137252.709 由重心决定
g=9.8;                     %重力加速度，单位为m/s^2
D=523*10^(-3);             %左右轮间距
Jy=160792.682*10^(-6);      %绕y轴的转动惯量 转向惯量  160792.682
Jx=207310.4*10^(-6);       %绕x轴的转动惯量

%状态空间参数初始化
a=r*(M+2*m+2*I/(r^2));
b=M*r*l;
c=Jz+M*l^2;
d=M*g*l;
e=M*l;
f=1/(r*(m*D+I*D/(r^2)+2*Jy/D));
A23=-b*d/(a*c-b*e);
A43=a*d/(a*c-b*e);
B21=(c+b)/(a*c-b*e);
B22=(c+b)/(a*c-b*e);
B41=-(e+a)/(a*c-b*e);
B42=-(e+a)/(a*c-b*e);
B61=f;
B62=-f;
%状态空间矩阵
A=[0 1  0  0 0 0;
   0 0 A23 0 0 0;
   0 0  0  1 0 0;
   0 0 A43 0 0 0;
   0 0  0  0 0 1;
   0 0  0  0 0 0]                           %状态矩阵
B=[0 0;B21 B22;0 0;B41 B42;0 0;B61 B62]     %输入矩阵

%系统可控性判断
Co=ctrb(A,B);
if(rank(Co)==6)
    disp('系统可控');
else
    disp('系统不可控');
end

%系统稳定性分析
[V,D]=eig(A);
y=diag(D)                               %系统特征值

%LQR控制器设计
Q=[1   0     0    0      0     0;
   0   20000   0    0      0     0;
   0   0   80000  0      0     0;
   0   0     0    5000    0     0;
   0   0     0    0      1     0;
   0   0     0    0      0     0];     %Q矩阵，三个参数分别影响pitch角度、角速度、车轮角速度的收敛速度
%。Q 矩阵的某一分量值越大，则该分量对应的状态变量以更快的速度衰减至0
%位移 位移速度 角度 角加速度 转向 转向速度
% 轮速度，pitch轴          轮速度，pitch轴      YAW轴角度、角速度
R=[1000 0;0 1000];                     %R矩阵，影响输入量的大小
K=lqr(A,B,Q,R)                         %调节参数

V=0;      %目标速度
Yaw=0;  %目标转角
position=1;%目标位置

% 创建一个示例表格
%load patients.mat
%T = table(LastName,Age,Weight,Smoker);
K_OUT=K(1,:);
% 将表格 T 写入到名为 patientdata.xlsx 的文件中的第一张工作表，并从单元格 D1 处开始
% filename = 'text2.xlsx';
% writematrix(K_OUT,filename,'Sheet',1,'Range','B1')



if i==1 %在开头输出一次Q矩阵
writematrix(Q,filename,'Sheet',1,'Range','I1')

end

    % 拼接数据范围字符串，如 'A1:F1'，'A2:F2' 等
    range = ['B',num2str(i),':G',num2str(i)];
    % 将矩阵写入到文件中的第一张工作表，并指定数据范围
    xlswrite(filename,K_OUT,1,range);
    range = ['A',num2str(i)];
    writematrix((133+i),filename,'Sheet',1,'Range',range);


    
end

