% Modelisation interactions de robots. 
%Programme principal à completer pour le TP 
%G. Chiavassa 2021

clear all;

%Nb individus
N=100;  

%Parametres de chaque individu
Mass=zeros(1,N); %masse
Relax=zeros(1,N); %temps de relaxation
Size=zeros(1,N);  %taille (rayon ri ds le cours)

% valeurs parametres des robots, tous égaux 
Mass(1,:)=0.5;    %en kg
Relax(1,:)=0.1;   %en sec
Size(1,:)=0.05;   %rayon en m
Vrobot=0.3;       %vitesse max d'un robot
Vpreda=1;         %vitesse max preda

%Parametres géométrie de la simulation
%domaine rectangulaire [0,a]x[0,b] 
a=10;
b=10;
c=4;
%x=[0:0.1:a];
%y=[0:0.1:b];
%z=[-c:0.1:c];

% Paramètres temps de simulation
Tfinal=17;
dt=0.08;


%Vecteurs et matrices utilisés (dimension 3 eventuellement)
Vn=zeros(3,N); %vitesses au temps tn
Vn1=zeros(3,N); %vitesses au temps tn+1
Vexpec=zeros(3,N); %vitesses désirées au temps tn
V_preda=[0.1;0.1;0.1];

Xn=zeros(3,N); %positions au temps tn
Xn1=zeros(3,N); %positions au temps tn+1

Force_others=zeros(3,N);

source=[3/4*a;1/2*b;1/2*c];
X_disque=[1/2*a;1/2*b;0];
r_disque=2;
X_predator=[a/2;b/2;c/2];
Size_preda=2;

%Initialisation
Xn(1,:)= a*rand(1,N)/6; 
Xn(2,:)=b*rand(1,N);
Xn(3,:)=c*rand(1,N);

t=0;
cont=1;

          %%%%%%%%%% Boucle en temps %%%%%%%%%%%%%%%
while (t<Tfinal)

   %source=[3/4*a*(5-sin(t+dt/4))/5 ; 1/2*b*(10-cos(t))/10 ; 1/2*c*(10-cos(t))/10];
   
   %centre de masse
   G=sum(Xn')'/N; 
    
  %%%%%%%%%% Calcul forces interaction avec les autres %%%%%%%%%%
  for i=1:N
    %calcul des forces s'execant sur robot i
    Force_others(:,i)=Interaction_robots(i,Xn,Vn,G,Size,N);
    
    %Vitesse expected du robot i
    Vexpec(:,i)=Vexpected_robots(i,Xn,Vn,source,Vrobot);
    
    %Force qui repousse les poissons par le prédateur
    Force_predator(:,i)=Interaction_predator(i,Xn,X_predator,Size,Size_preda);

    %Distance entre les poissons et le prédateur
    distance(i)=norm(X_predator-Xn(:,i));   

    %Force obstacle
    %Force_obstacle(:,i)=Interaction_disque(i,Xn,X_disque,Size(i),r_disque);
  end

  %Poisson le plus proche du prédateur
   [proche_distance,proche_indice]=min(distance);
   proche_direction=X_predator-Xn(:,proche_indice)/proche_distance;

   V_preda = V_preda + Vpreda*proche_direction;

 %Itération vitesses, schéma d'Euler explicite
 
   Vn1=Vn + dt*(Vexpec-Vn)./Relax + dt * Force_others./Mass + dt*Force_predator./Mass;%+ dt*Force_obstacle./Mass;
  
%encadrement de la vitesse, juste pour des questions de stabilité à
%l'initalisation
   Vn1=min(2*Vrobot,Vn1); Vn1=max(-2*Vrobot,Vn1);
  
%calcul des nouvelles positions

   Xn1=Xn + dt*Vn1;

   X_predator=X_predator + dt*V_preda;

%%%%%   calcul de differentes caractéristiques du mouvement

%2) Norme vitesse moyenne: 
   for i=1:N
    Norme(i)=norm(Vn1(:,i));
   end
Moyenne(cont)=mean(Norme);




%3) Modification de la position de la source au cours du temps:  A FAIRE
 
%5) Distance moyenne entre les robots: 
% for i=1:N
%     for j=1:N;
%         if j!=i;
%             NormeX[i]=norm(Xn1[:;i;0]-Xn1[:;j;0]);
% 
% end
% A FAIRE




    %%%%%%%   Tracé de la position des robots au temps t %%%%%
    %%%%%%%   toutes les nbiter itérations   %%%%%%
  nbiter=2;
  
  if mod(cont,nbiter)==0 
    figure(1)
    clf();
    % z = peaks(25);
    % mesh(z)
    hold on
   %tout les robots de la meme couleur
    %plot3(Xn1(1,:),Xn1(2,:),Xn1(3,:),'bo','MarkerSize',5,'MarkerFaceColor','b');
    scatter3(Xn1(1,:),Xn1(2,:),Xn1(3,:),10,'filled','blue');

   %robot 1 en rouge si leader
   %plot(Xn1(1,1),Xn1(2,1),'ro','MarkerSize',5,'MarkerFaceColor','r');

   %tracé de la source si elle existe
   %plot3(source(1,1),source(2,1),source(3,1),'gd','MarkerSize',8,'MarkerFaceColor','g');
   scatter3(source(1,1),source(2,1),source(3,1),'filled','green');

   %[x,y,z]=cylinder(r_disque,100);
   %surf(x+X_disque(1),y+X_disque(2),z*10+X_disque(3),);

   %predator
   scatter3(X_predator(1),X_predator(2),X_predator(3),200,'filled','red');

   axis equal
   grid on 
   view(10,10)

   %Soit dans le domaine [0,a]x[0,b]
   xlim([0 a]);
   ylim([0 b]);
   zlim([0 c]);
   %plot3([0,a],[0,0],,'k','LineWidth',2);
   %plot3([0,a],[b,b],'k','LineWidth',2);
   txt=['t = ',num2str(t)];
   text(a/2,b+b/20,txt,'Fontsize',12);

   %%soit autour du centre de masse
   % ddx=max(Xn1(1,:))-min(Xn1(1,:));
   % ddy=max(Xn1(2,:))-min(Xn1(2,:));
   % xlim([G(1,1)-ddx G(1,1)+ddx]);
   % ylim([G(2,1)-ddy G(2,1)+ddy]);
   % txt=['t = ',num2str(t)];
   % text(G(1,1),G(2,1)+ddy,txt,'Fontsize',12);

   % hold off
   % drawnow limitrate nocallbacks;
    end %fin end du if du tracé
% fin du tracé

   %%%%% mise a jour des variables pour itération suivante %%%%
Vn=Vn1;
Xn=Xn1;
t=t+dt;
temps(cont)=t;
cont=cont+1;
end

%%figure variation vitesse moyenne en fct du temps
% figure(2)
% plot(temps,Moyenne);
% xlabel('time sec');
% ylabel('Vitesse moyenne m/sec)');
% A FAIRE
  
%%figure variation distance minimale moyenne en fct du temps
  % A FAIRE
