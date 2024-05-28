clear all;

%Nb individus
N=50;  

%Parametres de chaque individu
Mass=zeros(1,N); %masse
Relax=zeros(1,N); %temps de relaxation
Size=zeros(1,N);  %taille (rayon ri ds le cours)

% valeurs parametres des robots, tous égaux 
Mass(1,:)=0.5;    %en kg
Relax(1,:)=0.1;   %en sec
Size(1,:)=0.05;   %rayon en m
Vpoisson=0.6;       %vitesse max d'un robot

%Parametres géométrie de la simulation
%domaine rectangulaire [0,a]x[0,b] 
a=2;
b=2;
c=2;
x=[0:0.1:a];
y=[0:0.1:b];
z=[0:0.1:c];

% Paramètres temps de simulation
Tfinal=30;
dt=0.08;


%Vecteurs et matrices utilisés (dimension 3 eventuellement)
Vn=zeros(3,N); %vitesses au temps tn
Vn1=zeros(3,N); %vitesses au temps tn+1
Vexpec=zeros(3,N); %vitesses désirées au temps tn

Xn=zeros(3,N); %positions au temps tn
Xn1=zeros(3,N); %positions au temps tn+1

Force_others=zeros(3,N);
Force_individual=zeros(3,N);

source=zeros(3,1);

%Initialisation
Xn(1,:)=a*rand(1,N)/6; 
Xn(2,:)=b*rand(1,N);
Xn(3,:)=c*rand(1,N);

t=0;
cont=1;

          %%%%%%%%%% Boucle en temps %%%%%%%%%%%%%%%
while (t<Tfinal)

   %centre de masse
   G=sum(Xn')'/N; 
    
  %%%%%%%%%% Calcul forces interaction avec les autres %%%%%%%%%%
  for i=1:N
    %calcul des forces s'exercant sur robot i
    Force_others(:,i)=Interaction_poissons(i,Xn,Vn,G,Size,N);
    Force_individual(:,i)=Force_locomotrice(i,Vn);

  end

 %Itération vitesses, schéma d'Euler explicite
 
   Vn1=Vn + dt*(Vexpec-Vn)./Relax + dt * Force_others./Mass;
  
%encadrement de la vitesse, juste pour des questions de stabilité à
%l'initalisation
   Vn1=min(2*Vpoisson,Vn1); Vn1=max(-2*Vpoisson,Vn1);
  
%calcul des nouvelles positions

   Xn1=Xn + dt*Vn1;

%%%%%   calcul de differentes caractéristiques du mouvement

%2) Norme vitesse moyenne :

%3) Modification de la position de la source au cours du temps:

source(1,1)=a;
source(2,1)=b/2;

 
%5) Distance moyenne entre les robots:  






    %%%%%%%   Tracé de la position des robots au temps t %%%%%
    %%%%%%%   toutes les nbiter itérations   %%%%%%
  nbiter=2;
  
  if mod(cont,nbiter)==0 
    figure(1)
    clf();
    hold on
   %tout les robots de la meme couleur
    plot(Xn1(1,:),Xn1(2,:),'bo','MarkerSize',5,'MarkerFaceColor','b');

   %robot 1 en rouge si leader
   %plot(Xn1(1,1),Xn1(2,1),'ro','MarkerSize',5,'MarkerFaceColor','r');

   %tracé de la source si elle existe
   plot(source(1,1),source(2,1),'gd','MarkerSize',8,'MarkerFaceColor','g');

   axis equal

   %Soit dans le domaine [0,a]x[0,b]
   xlim([0 a]);
   ylim([0 b]);
   plot([0,a],[0,0],'k','LineWidth',2);
   plot([0,a],[b,b],'k','LineWidth',2);
   txt=['t = ',num2str(t)];
   text(a/2,b+b/20,txt,'Fontsize',12);

   %%soit autour du centre de masse
   % ddx=max(Xn1(1,:))-min(Xn1(1,:));
   % ddy=max(Xn1(2,:))-min(Xn1(2,:));
   % xlim([G(1,1)-ddx G(1,1)+ddx]);
   % ylim([G(2,1)-ddy G(2,1)+ddy]);
   % txt=['t = ',num2str(t)];
   % text(G(1,1),G(2,1)+ddy,txt,'Fontsize',12);

   hold off
   drawnow limitrate nocallbacks;
   end %fin end du if du tracé
% fin du tracé

   %%%%% mise a jour des variables pour itération suivante %%%%
Vn=Vn1;
Xn=Xn1;
t=t+dt;
temps(cont)=t;
cont=cont+1;
end
