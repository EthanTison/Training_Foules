function F=Interaction_poissons(I,X,V,Gmasse,Size,N)
%calcul de la force qui s'exerce sur l'individu I de la part des autres et
%du groupe

%constantes multiplicatives devant les forces
CO=0.5;
CG=0.05;

F=[0;0;0];
alpha=0.5;   %constante entre 0 et 1

for j=1:N
    if not(j == I)
        l0=5*(Size(I)+Size(j));
        normale=(X(:,I)-X(:,j))/l0;
        d=norm(normale);

        % Vecteur vitesse du poisson i
    VIx = V(1,I);
    VIy = V(2,I);
    VIz = V(3,I);
    
    % Position des poissons i et j
    XIx = X(1,I);
    XIy = X(2,I);
    XIz = X(3,I);
    
    Xjx = X(1,j);
    Xjy = X(2,j);
    Xjz = X(3,j);
    
    % Vecteur qui relie le poisson i au poisson j
    XIj = [Xjx - XIx, Xjy - XIy, Xjz - XIz];
    
    % Projections sur le plan XY
    VIXY = [VIx, VIy, 0];
    XIjXY = [XIj(1), XIj(2), 0];
    
    % Projections sur le plan XZ
    VIXZ = [VIx, 0, VIz];
    XIjXZ = [XIj(1), 0, XIj(3)];
    
    % Calcul des angles
    thetaXY = acos(dot(VIXY, XIjXY) / (norm(VIXY) * norm(XIjXY)));
    thetaXZ = acos(dot(VIXZ, XIjXZ) / (norm(VIXZ) * norm(XIjXZ)));

    aniso= 1 + alpha*cos(thetaXY)*cos(thetaXZ);
    
    F= F + aniso*CO*normale*(d^(-3)-d^(-2))*exp(-d);
    end
end

 F=F+ CG*(Gmasse-X(:,I))/norm(Gmasse-X(:,I))/N;
 
end
*
