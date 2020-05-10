d_a=10;
C_x=0;
C_y=0;
i=0;
r=1
alpha=0;
for alpha=0:d_a:pi/2
    x(i)=C_x-r+r*cos(alpha);
    y(i)=C_y+r+r*sin(alpha);
    i=i+1;
end 
plot(x,y)