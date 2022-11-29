
close all
%---- Données necessaires pour ajouter la 3D ----%

    %Les points 3D suivis
load('X.mat')
load('Y.mat')
    %L'image à insérer
imageInserer=imread('Chat.jpg');

    %La vidéo de l'énoncé
videoInfos = VideoReader("video_base.mp4");

%---- Lancement de l'insertion de l'image ----%

videoChat = Remplacement(imageInserer,videoInfos,X,Y);

%---- Les fonctions appelées ----%

function video = Remplacement(imageInserer,videoInfos,X,Y) 
%Fonction principale de l'homographie 2D, elle coordonne toutes les
%fonctions implémmentées et renvoie la video avec l'image inséré.

    %Seuillage empirique pour le masque de la main
    seuil=130;
    %On va venir recupérer les coordonnées réelles des points suivis
    %dans l'image insérée
    [X2,Y2] = RecupPointsImageInserer(imageInserer);

    %Création d'un objet de la classe VideoWriter. Base pour créer notre
    %vidéo
    video = VideoWriter('videoChat');
    %Ouverture de la vidéo pour la modifier
    open(video)

    %Parcours de toutes la vidéo frame par frame
    for i=1:videoInfos.NumFrames
        %Calcul de la matrice d'homographie pour l'image i
        H = Homographie2D(X(i,:), Y(i,:), X2, Y2);
        %Lecture de la ième frame de la vidéo
        frame=read(videoInfos,i);
        %Création de la frame à ajouter à la vidéo
        newFrame=Insertion(frame,imageInserer,H,X(i,:),Y(i,:),seuil);
        imshow(newFrame)
        %Ajout de frame renvoyé par Insertion à la vidéo 
        writeVideo(video,newFrame);
    end
    close(video)
end

function [X,Y] = RecupPointsImageInserer(imageInserer)
%Recuperation des points de l'image à inserer correspondant aux points
%suivis rangé dans l'ordre de sélection du détecteur de Harris. 

    [nblignes,nbcolonnes,~]=size(imageInserer);

    %Les coordonnées des points associé aux points suivis
    X = [0 nbcolonnes 0 nbcolonnes nbcolonnes/2];
    Y = [0 0 nblignes nblignes nblignes/2];
end

function H = Homographie2D(X1, Y1, X2, Y2)
% HOMOGRAPHIE : cette fonction retourne la matrice d'homographie H permettant
% le passage d'une image dans la feuille papier à une image dans un quadrilatère
% X1: est la liste des coordonnées x des points de la feuille
% Y1: est la liste des coordonnées y des points de la feuille
% X2: est la liste des coordonnées x des points de l'image à insérer
% Y2: est la liste des coordonnées y des points de l'image à insérer 

% On suppose que les 4 vecteurs en entrée sont de longueur supérieure
% ou égale à 4.

    % k represente un duo de points
    k=1; 
    % N est le nombre de points pris en compte pour calculer la matrice d'homographie
    N=length(X2); 
    X=zeros(N,1);%Matrice colonne des coefficients de l'homographie
    A=zeros(size(X,1), 8);
    B=zeros(size(X,1),1);
    
    % Calcul des coefficients des matrices A et B
    for i = 1 : 2*N
        %Initialisation des coefficients des matrices A et B
        if(mod(i,2)~=0)
            A(i,:)=[X1(k) Y1(k) 1 0 0 0 -X1(k)*X2(k) -Y1(k)*X2(k)];
            B(i,1)=X2(k);
        else
            A(i,:)=[0 0 0 X1(k) Y1(k) 1 -X1(k)*Y2(k) -Y1(k)*Y2(k)];
            B(i,1)=Y2(k);
            k = k+1; %On change de duo de points
        end
    end
    
    % Calcul des coefficients de la matrice X
    X=A\B;
    
    %Mise en forme de la matrice d'homographie
    H=[X(1) X(2) X(3);
       X(4) X(5) X(6);
       X(7) X(8) 1];
end

function newframe = Insertion(frame,imgainserer,H,X,Y,seuil)
%Insertion de l'image sur une frame de la video
    
    %Récupération des pixels de la frame
    newframe=frame;

    %Chercher les coordonnées du carré à modifier autour de la feuille
    Ymax = round(max(Y));
    Ymin = round(min(Y));
    Xmax = round(max(X));
    Xmin = round(min(X));

    %Taille de l'image insérée
    largeurImg=size(imgainserer,2);
    hauteurImg=size(imgainserer,1);

    for x1 = Xmin:Xmax
        for y1 = Ymin:Ymax 
            %Création d'un point X2 en coordonnées homogènes
            %Coordonnées dans le repère de la vidéo
            X1=[x1;y1;1];
            X2=H*X1; 
            X2=X2./X2(3); % Normalisation
            x2= round(X2(1));
            y2= round(X2(2));

            %Vérification que le point à insérer est dans l'image
            %Attention x,y dans matlab différent des coordonnées dans
            %une image
            if(x2>0 && x2<=largeurImg && y2>0 && y2<=hauteurImg)
                %On regarde si le point est recouvert par la main
                masque=MasqueMain(frame,x2,y2,x1,y1,seuil);
                if(masque == 0)
                    %Il n'est pas recouvert on peut insérer l'image
                    newframe(x1,y1,:)= imgainserer(y2,x2,:);%[0 0 0]; 
                end   
            end
        end 
    end
end

function filtre = MasqueMain(frame,x,y,i,j,seuil)
% En un point donné ce filtre renvoie 1 si on est sur la main 0 sinon

    %Coordonnées dans l'image à insérer définies empiriquement comme support de
    %calcul du masque de la main
    X=[850 1200];%largeur du support
    Y=[50 550];%hauteur du support
    
    filtre = 0 ; %Initialisation
    %Calcul du filtre uniquement dans un voisinage empirique de la main
    if(x>X(1) && x<=X(2) && y>Y(1) && y<=Y(2))
        Rouge = frame(i,j,1);
        Vert = frame(i,j,2);
        Bleu = frame(i,j,3);
                            
        %On utilise le Cr pour détecter la main car c'est ce qui rend la main la
        %plus contrastée par rapport à la feuille en entier
        Cr = 0.5*Rouge-0.4187*Vert - 0.0813*Bleu+128;

        %Test booléen
        filtre = Cr > seuil;
    end
end

function  [x1,y1] = Interpolation(x1,y1)
% Cette fonction renvoie un arrondi des coordonnées entrées, en utilisant
% l'interpolation
% Calculs des aires des 4 rectangles séparant le point des points de
% coordonnées entières l'entourant.
        A11=(x1-floor(x1))*(y1-floor(y1));
        A12=(x1-floor(x1))*(ceil(y1)-y1);
        A21=(ceil(x1)-x1)*(y1-floor(y1));
        A22=(ceil(x1)-x1)*(ceil(y1)-y1);
        A=[A11 A12 A21 A22];
        Amin=min(A); % Recherche du rectangle d'aire minimum
        % On remplace les coordonnées x1 et y1 par celles du point le plus
        % proche, pour lequel l'aire du rectangle est minimale
        if(Amin==A11)
            x1=floor(x1);
            y1=floor(y1);
        elseif(Amin==A12)
            x1=floor(x1);
            y1=ceil(y1);
        elseif(Amin==A21)
            x1=ceil(x1);
            y1=floor(y1);
        elseif(Amin==A22)
            x1=ceil(x1);
            y1=ceil(y1);
        end
end
















