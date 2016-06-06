p = [0 10 20 30 40 50 60 70 80 90];
v = [1 3 5 8 6 5 4 3 2 1];
for i=1:5
    ax.joint(i,0,10)
end
pause()
for i=1:10
ax.joint(3,p(i),v(i))
while ax.moving == 1
pause(0.001)
end
end