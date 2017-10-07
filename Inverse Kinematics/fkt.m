function res = fkt(link_len, t)

l1 = link_len(1);
l2 = link_len(2);
l3 = link_len(3);

xee  = l1*cos(t(1)) + l2*cos(t(1)+t(2)) + l3*cos(t(1)+t(2)+t(3));
yee  = l1*sin(t(1)) + l2*sin(t(1)+t(2)) + l3*sin(t(1)+t(2)+t(3));

res = [xee yee];
end 