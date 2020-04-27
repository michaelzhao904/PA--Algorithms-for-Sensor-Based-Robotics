function inv = inv_T(T)
R = T(1:3,1:3);
p = T(1:3,4);
inv = [R' -R'*p;
         zeros(1,3), 1];
end
