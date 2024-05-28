function FIloc=Force_locomotrice(I,V)

Vm=0.3;
k=0.5;

FIloc=k*(1-((norm(V(:,I)))^2/Vm^2))*V(:,I);

end
