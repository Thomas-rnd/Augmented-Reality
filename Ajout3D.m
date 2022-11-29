close all
%---- Données necessaires pour ajouter la 3D ----%

    %Les points suivis de la vidéo
load('X.mat')
load('Y.mat')

    %La vidéo avec l'image déjà insérée
videoInfos = VideoReader("videoChat.avi");

    %Notre modèle 3D à insérer
%Caractéristiques
tailleXYZ=[sqrt(2)*0.1 0.1 0.1];
%Centre pour translation suivant x et z
centre=[0 0.5 0.5];
%Centre pour translation suivant z uniquement
%centre=[sqrt(2)/2 0.5 0.2];

%Création des modèles suivant les caractéristiques choisies
%[points,segments,centre]=PaveDroit(tailleXYZ(1),tailleXYZ(2),tailleXYZ(3),centre);
[points,segments,centre]=Sphere3D(0.1,centre);

%---- Lancement du programme ----%
video3D = Remplacement3D(videoInfos,centre,X,Y); 

%---- Ensembles des fonctions appelées ----%
    %---- MODELES 3D ----%

function [points,segments,centre] = PaveDroit(x,y,z,centre)
%Cette fonction renvoie les informations necessaires au dessin
%d'un pavé droit. Un pavé droit est constitué de 8 points reliés 
%via 12 segments.
%On le construit suivant les coordonnées de son centre

%Choix de numérotation des points : numéroté dans le sens des aiguilles
%d'une montre en partant la base jusqu'au plan haut du pavé (on part du point en haut 
%à gauche pour chacun des 2 plans. 

%Matrice 8x3 : 1 point par ligne suivant les 3 dimensions de l'espace
%Le centre et la base respecte la base choisie
    points=[centre(1)-x/2  centre(2)-y/2 centre(3)-z/2;
            centre(1)+x/2  centre(2)-y/2 centre(3)-z/2;
            centre(1)+x/2  centre(2)+y/2 centre(3)-z/2;
            centre(1)-x/2  centre(2)+y/2 centre(3)-z/2;
            centre(1)-x/2  centre(2)-y/2 centre(3)+z/2;
            centre(1)+x/2  centre(2)-y/2 centre(3)+z/2;
            centre(1)+x/2  centre(2)+y/2 centre(3)+z/2;
            centre(1)-x/2  centre(2)+y/2 centre(3)+z/2;];

    segments=[1 2;
              2 3;
              3 4;
              4 1;
              5 6;
              6 7;
              7 8;
              8 5;
              1 5;
              2 6;
              3 7;
              4 8];
end

function [points,segments,centre] = Sphere3D(rayon,centre)
%Création d'un modèle 3D d'une sphère en fonction de son rayon et de son
%centre.

    [X,Y,Z] = sphere(9);
    % [X,Y,Z] = sphere renvoie les coordonnées x, y, z d'une sphère sans
    % l'afficher. Par défaut la sphère est de rayon 1. Lui mettre un 9 en 
    % argument donnera une sphère échantillonnée de 9x9 faces. (74 points)

    X = X * rayon + centre(1); 
    Y = Y * rayon + centre(2);
    Z = Z * rayon + centre(3); 
    % Ces lignes permettent de modifier le rayon et les coordonnées du
    % centre de la sphère (par defaut de rayon 1 et de centre (0,0,0)).

    X=reshape(X(:,1:9),90,1);
    Y=reshape(Y(:,1:9),90,1);
    Z=reshape(Z(:,1:9),90,1);

    % Par défaut, [X Y Z]=sphere() renvoie trois matrices 9x9 : une pour
    % toutes les coordonnées en X, une pour toutes les coordonnées en Y,
    % une pour toutes les coordonnées en Z. 
    % "reshape" permet de passer ces trois matrices carrées en matrices
    % colonne avec 90 lignes et 1 colonne (on met les colonnes les unes en
    % dessous des autres).

    % Il y a 90 lignes et pas 74 car le point du sommet et de la base de la sphère sont 
    % répétés à chaque demi-cercle parce que la fonction sphere les remet 
    % comme point d'arrivée et de départ pour chaque demi-cercle.

    points=[X Y Z]; % Il suffit ensuite de concaténer ces 3 matrices pour 
                    % avoir une matrice 90x3 avec les coordonnées dans
                    % l'espaces de tous les points de la sphère

 
    % Pour les segments on reprend indices des lignes de la matrice points
    % et on relie les points qu'il faut entre eux.

    % La fonction [X Y Z]=sphere() part du point en bas de la sphère et 
    % avant d'appliquer le reshape, chaque colonne  correspondait aux coordonnées 
    % de tous les points sur un des 9 demi-cercles à la verticale.
    % Ensuite elle balaie tous les autres demi-cercles un par un. 

    % Donc après le reshape, les points du 2ème demi-cercle (ou de 
    % l'ancienne 2ème colonne) vont prendre les lignes de 11 à 20. 

    
    segments = [1 2; % segments des 9 demi-cercles verticaux
                2 3; 
                3 4;
                4 5;
                5 6;
                6 7;
                7 8;
                8 9;
                9 10;
                
                11 12; % le 11 est le même point que le 1 (base de la sphère)
                12 13;
                13 14;
                14 15;
                15 16;
                16 17;
                17 18;
                18 19;
                19 20; % le 20 est le même point que le 10 (sommet de la sphère)

                21 22;
                22 23;
                23 24;
                24 25;
                25 26;
                26 27;
                27 28;
                28 29;
                29 30;
                
                31 32;
                32 33;
                33 34;
                34 35;
                35 36;
                36 37;
                37 38;
                38 39;
                39 40;
                
                41 42;
                42 43;
                43 44;
                44 45;
                45 46;
                46 47;
                47 48;
                48 49;
                49 50;
                
                51 52;
                52 53;
                53 54;
                54 55;
                55 56;
                56 57;
                57 58;
                58 59;
                59 60;
                
                61 62;
                62 63;
                63 64;
                64 65;
                65 66;
                66 67;
                67 68;
                68 69;
                69 70;
                
                70 71;
                71 72;
                72 73;
                73 74;
                74 75;
                75 76;
                76 77;
                77 78;
                78 79;
                79 80;
                
                81 82;
                82 83;
                83 84;
                84 85;
                85 86;
                86 87;
                87 88;
                88 89;
                89 90;
                
                2 12; % segments entre les points des 8 cercles horizontaux
                12 22;
                22 32;
                32 42;
                42 52;
                52 62;
                62 72;
                72 82;
                82 2;

                3 13;
                13 23;
                23 33;
                33 43;
                43 53;
                53 63;
                63 73;
                73 83;
                83 3;

                4 14;
                14 24;
                24 34;
                34 44;
                44 54;
                54 64;
                64 74;
                74 84;
                84 4;

                5 15;
                15 25;
                25 35;
                35 45;
                45 55;
                55 65;
                65 75;
                75 85;
                85 5;

                6 16;
                16 26;
                26 36;
                36 46;
                46 56;
                56 66;
                66 76;
                76 86;
                86 6;

                7 17;
                17 27;
                27 37;
                37 47;
                47 57;
                57 67;
                67 77;
                77 87;
                87 7;

                8 18;
                18 28;
                28 38;
                38 48;
                48 58;
                58 68;
                68 78;
                78 88;
                88 8;

                9 19;
                19 29;
                29 39;
                39 49;
                49 59;
                59 69;
                69 79;
                79 89;
                89 9;
                ];
end

function video = Remplacement3D(videoInfos,centre,X,Y) 
%Fonction principale de l'insertion 3D, elle coordonne toutes les
%fonctions implémentées et renvoie la video avec l'objet 3D inséré.
    
    %Initialisation du repère de base pour la 3D définis dans l'énoncé
    [X1,Y1,Z1] = InitialisationBase3D();

    %Création d'un objet de la classe VideoWriter
    video = VideoWriter('videoBoite'); 
    %Ouverture de la vidéo pour la modifier
    open(video)
    for i=1:videoInfos.Numframes
        %Calcul de la matrice d'homographie pour l'image i
        P = Transformation3D(X1,Y1,Z1,X(i,:),Y(i,:));
        
        %Lecture de la ième frame de la vidéo
        frame=read(videoInfos,i);

        %Ajout d'une translation au centre de l'objet 3D uniquement. 
        %Elle est composé d'une composante x et z (l'ajout de la composante y est similaire)
        centre=TranslationX(centre,i,videoInfos.Numframes);
        centre=TranslationZ(centre,i,videoInfos.Numframes);
        %Création d'un objet 3D en fonction de son centre translaté
        %[points,segments,centre]=PaveDroit(0.2,0.2,0.2,centre);
        [points,segments,centre]=Sphere3D(0.1,centre);

        %Ajout d'une rotation à l'objet 3D suivant x,y et/ou z
        angleZ=i/20;
        angleY=i/10;
        angleX=1;
        points=Rotation(angleZ,angleY,angleX,points,centre);

        %Création de la frame à ajouter à la vidéo
        newFrame=Insertion3D(frame,points,segments,P);
        imshow(newFrame)

        %Ajout de frame renvoyé par Insertion3D à la vidéo 
        writeVideo(video,newFrame);
    end
    close(video)
end

function [X1,Y1,Z1] = InitialisationBase3D()
%Initialisation des points que nous suivons dans la vidéo en donnant
%leurs coordonnées dans le repère 3D donnée

    X1=[0 sqrt(2) 0 sqrt(2) sqrt(2)/2 0.350*sqrt(2) 0.55*sqrt(2) 0.125*sqrt(2) 0.4*sqrt(2)];
    Y1=[0 0 1 1 0.5 0.775 0.45 0.55 0.55];
    Z1=[0 0 0 0 0 0.2 0.3 0.2 0.2];
end

function [P] = Transformation3D(X1, Y1, Z1, X2, Y2)
% HOMOGRAPHIE : cette fonction retourne la matrice d'homographie P
% X1: est la liste des coordonnées x1 des points dans notre repère 3D
% Y1: est la liste des coordonnées y1 des points dans notre repère 3D
% Z1: est la liste des coordonnées z1 des points dans notre repère 3D 
% X2: est la liste des coordonnées x2 des points de la feuille
% Y2: est la liste des coordonnées y2 des points de la feuille
    
    k=1; % k represente un duo de points
    N=length(X2); % N est le nombre de points pris en compte pour calculer la matrice d'homographie
    A=zeros(N, 11);
    B=zeros(N,1);

    % i représente une ligne de la matrice
    for i = 1 : 2*N
        if(mod(i,2)~=0)
            A(i,:)=[X1(k) Y1(k) Z1(k) 1 0 0 0 0 -X1(k)*X2(k) -Y1(k)*X2(k) -Z1(k)*X2(k)];
            B(i,1)=X2(k);
        else
            A(i,:)=[0 0 0 0 X1(k) Y1(k) Z1(k) 1 -X1(k)*Y2(k) -Y1(k)*Y2(k) -Z1(k)*Y2(k)];
            B(i,1)=Y2(k);
            k = k+1; %On change de duo de points
        end
    end

    % Calcul des coefficients de la matrice X
    X=A\B;

    %Génération de la matrice P en réorganisant la matrice X
    P=[X(1) X(2) X(3) X(4);
       X(5) X(6) X(7) X(8);
       X(9) X(10) X(11) 1];

end

function [point] = TranslationX(point,i,numFrames)
%Translation d'un point entre 0 et sqrt(2) suivant x 
    
    x=sqrt(2)*i/numFrames;
    point(:,1)=x;
end

function [point] = TranslationZ(point,i,numFrames)
%Equation du rebond d'une balle simulé par une fonction trigonométrique
%Valeurs comprises entre 0 et 1 suivant l'axe z

    x=sqrt(2)*i/numFrames;
    fx=(cos(11*x)*cos(11*x)/(2*x+1));  
    point(:,3)=fx;  
end

function [points] = Rotation(angleZ,angleY,angleX,points,centre)
%Application d'une rotation à un objet suivant une base propre à cet objet
%ayant pour origine son centre. Pas de modification de la position du
%centre dans le repère de la feuille. 

    %Convertion des coordonnées de la base de la feuille dans une 
    %base orthonormée ayant pour origine le centre de l'objet. 
    %Cette conversion permet à l'objet de tournée suivant l'axe verticale
    %ayant pour origine son centre. 
    points=points-centre;

    %Génération de la matrice de rotation suivant l'axe z
    Rz=[cos(angleZ) -sin(angleZ) 0;
       sin(angleZ) cos(angleZ) 0;
       0 0 1];
    %Génération de la matrice de rotation suivant l'axe y
    Ry=[cos(angleY) 0 sin(angleY);
       0 1 0;
       -sin(angleY) 0 cos(angleY)];
    %Génération de la matrice de rotation suivant l'axe x
    Rx=[1 0 0;
        0 cos(angleX) -sin(angleX);
        0 sin(angleX) cos(angleX)];

    %Rotation de l'objet suivant l'axe z
    points=points*Rz;
    %Rotation de l'objet suivant l'axe y
    points=points*Ry;
    %Rotation de l'objet suivant l'axe x
    points=points*Rx;

    %Convertion des coordonnées de la base de l'objet dans la base de la
    %feuille pour l'afficher au bon endroit. 
    points=points+centre;
end

function [image] = Insertion3D(frame, points,segments, P)
%Remplace les pixels de l'image extraite de la vidéo d'origine par les
%pixels correspondant à l'objet 3D.
    
    %Données
    dimFrame = size(frame);
    points2D=[];
    
    %Calcul des coordonnées des points dans la feuille grâce à
    %l'homographie 3D
    for i = 1:size(points,1)
        %Création d'un vecteur en coordonnées homogènes d'un point 3D
        X1 = [points(i,1) ; points(i,2) ; points(i,3) ; 1];
        X2 = P*X1;
        X2=X2./X2(3); % Normalisation
        x2 = round(X2(1));
        y2 = round(X2(2));
        %Attention à ne pas confondre la dimension pour une image hauteur/largeur et x/y
        %pour une matrice
        %Vérification que le point 3D à afficher est dans l'image
        if (x2>0 && x2<dimFrame(1) && y2>0 && y2<dimFrame(2))        
            points2D=[points2D ; x2 y2];
        end
    end

    
    %Trace les segments entre la liste de points2D sur la frame
    %Epaisseur du fil de fer
    epaisseur=1;
    image = TraceEpais(points2D, segments, frame, epaisseur);
end

function [image] = TraceEpais(points2D, segments, frame, epaisseur)
%Trace les segments entre les points sur une image. Les segments prennent 
%la forme d'une succession de points échantillonnés. Plusieurs segment parallèle 
%sont tracé pour lui ajouter de l'épaisseur
 
    %paramètre de sur-échantillonnage
    alpha = 1;
    
    %Initialement l'image renvoyé n'a pas subit de modifications
    image=frame;
    
    %On parcoure tous les segments
    for i=1:size(segments,1)
        
        %Récupération des deux points considérés pour tracer le segment.
        %Les points sont deux matrices 1x2
        Pt1=points2D(segments(i,1),:);
        Pt2=points2D(segments(i,2),:);

        %Détermination du nombre de points « utiles »
        L=alpha*sqrt((Pt1(1)-Pt2(1))*(Pt1(1)-Pt2(1)) + (Pt1(2)-Pt2(2))*(Pt1(2)-Pt2(2)));

        %Détermination du vecteur normé orthogonal au segment Pt1Pt2
        nX=-(Pt2(2)-Pt1(2))/(sqrt((Pt1(1)-Pt2(1))*(Pt1(1)-Pt2(1)) + (Pt1(2)-Pt2(2))*(Pt1(2)-Pt2(2))));
        nY=-(Pt2(1)-Pt1(1))/(sqrt((Pt1(1)-Pt2(1))*(Pt1(1)-Pt2(1)) + (Pt1(2)-Pt2(2))*(Pt1(2)-Pt2(2))));
    
        %Ajout du segment sur la frame sous la forme de L points
        for j=1:L
            for k=-epaisseur:epaisseur
                %Création du point à modifier
                x=Pt1(1)+(j/(L-1))*(Pt2(1)-Pt1(1))+k*nX;
                y=Pt1(2)+(j/(L-1))*(Pt2(2)-Pt1(2))+k*nY;
        
                %Modification du point sur la frame en le coloriant en rouge
                if(x>0 && y>0)
                    image(round(x),round(y),:)= [255 0 0];%Couleur();
                end 
            end
        end 
    end
end

function [couleur] = Couleur()
%Génération d'un triplet RGB aléatoire
    couleur = randi([0 255],1,3);  
end



