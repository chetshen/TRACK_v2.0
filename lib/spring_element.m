function [K,C]=spring_element(k,c)

K=k*[1  0 -1 0
     0  0 0  0
     -1 0 1  0
     0  0 0  0];
 C=c*[1  0 -1 0
     0  0 0  0
     -1 0 1  0
     0  0 0  0];
end