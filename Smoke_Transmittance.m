function Smoke_Transmittance
trans=zeros(1000,3);
obj=VideoReader('3-5,3.avi');
img=read(obj,1);
img=img(150:450,100:870);
imshow(img),hold on;

x_label=390;
y_label=225;
plot(x_label,y_label,'ro','MarkerSize',15,'LineWidth',2);%(390,194),(432,194),(476,194)
grey = double(img(y_label,x_label))

%determing the max radiance target
double max_target;
max_target=0;
for i=0:19
    for j=0:19
          if img(y_label-10+i,x_label-10+j) > max_target
                   max_target = img(y_label-10+i,x_label-10+j);
                   x=x_label-10+i;y=y_label-10+j;   
          end
    end
end
max_target
x
y
plot(x,y,'go','MarkerSize',15,'LineWidth',2)
%Part 1£º determine the target radiance without smoke G1
% A=zeros(40,1); %horizontal
% for i=1:40
%    A(i)=img(194,370+i);  
% end
% A
% G1h = max(A)
% B=zeros(40,1);%vertical
% for j=1:40
%   B(j)=img(174+j,390);  
% end
% B
% G1v = max(B)
% G1 = max(G1h,G1v)
G1=double(img(y,x));

%Part 2£º determine the background radiance without smoke G2
G21=double(img(y,x+10));
G22=double(img(y+10,x));
G23=double(img(y,x-10));
G24=double(img(y-10,x));
G2a=G21;             
G2b=(G21+G22+G23+G24)/4;            

Bg=zeros(40,40);
for i=1:40
    for j=1:40
        Bg(i,j)=double(img(y-21+j,x-21+i));    
    end 
end
G2c=mean(Bg(:));

for i=1:1000                  %500 frames of video
obj=VideoReader('3-5,3.avi');
img=read(obj,i);
img=img(150:450,100:870);
%imshow(img),hold on;
%plot(390,194,'ro','MarkerSize',15,'LineWidth',2);
%a,b,c are three methods of selection points
G3=double(img(y,x));     %G3 means target radiance with smoke
G41=double(img(y,x+10));    %G4 means background radiance without smoke
G42=double(img(y+10,x));
G43=double(img(y,x-10));
G44=double(img(y-10,x));
G4a=G41;
G4b=(G41+G42+G43+G44)/4;

Bg1=zeros(40,40);
for k=1:40
    for n=1:40
        Bg1(k,n)=double(img(y-21+k,x-21+n));    
    end 
end
G4c=mean(Bg1(:));


Ta=(G3-G4a)/(G1-G2a);
Tb=(G3-G4b)/(G1-G2b);
Tc=(G3-G4c)/(G1-G2c);

trans(i,1)=Ta;
trans(i,2)=Tb;
trans(i,3)=Tc;
end
dlmwrite('smoke transmittance.xls',trans,'precision','%.2f','delimiter','\t');