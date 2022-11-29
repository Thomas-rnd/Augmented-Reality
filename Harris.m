close all
%---- Données necessaires pour détecter les points ----%

videoInfos = VideoReader("video_base.mp4");

%---- Lancement de la détection de points ----%

[X,Y] = DetectionPoints(videoInfos);

%---- Test de validité des points obtenus ----%

TestUnitaireTracking(X,Y,videoInfos);

%---- L'ensemble des fonctions appelées ----%

function [X,Y] = DetectionPoints(videoInfos)
%Fonction générale implémentant le détecteur de Harris
    
    %Constantes filtres gaussiens
    sigma_g=2;
    sigma_c1=3;
    sigma_c2=5;
    
    %Initialisation des points à suivre
    %Appuyer sur Entrer pour finir la sélection
    frame = read(videoInfos,1);
    imshow(frame);
    [yInit,xInit]= ginput;
    nbPoints=size(xInit,1);
    
    %Matrices des coordonnées des points suivi, 
    %chaque ligne représente une frame de la vidéo
    X=zeros(videoInfos.NumFrame,nbPoints);
    Y=zeros(videoInfos.NumFrame,nbPoints);
    
    %Taille de la fenêtre de recherche d'un maximum
    W=37;
    
    %Calcul de toutes les coordonnées X et Y
    for i=1:videoInfos.NumFrame
            %Extraction d'une frame de l'image
            frame = read(videoInfos,i);
    
            %Conversion en Noir et Blanc
            frameNB = rgb2gray(frame);
            %Figure de vérification
            %figure, imshow(frameNB)
    
            %Calcul des matrices de covariances
            DeriveX=conv2(frameNB,FiltreG('X',sigma_g),'same');
            DeriveY=conv2(frameNB,FiltreG('Y',sigma_g),'same');
            
            %Figures de vérification
            %figure,imshow(DeriveX,[0,10]),colormap(flipud(gray(256)))
            %figure,imshow(DeriveY,[0,10]),colormap(flipud(gray(256)))
    
            %Vérification de la norme de la dérivée des deux points
            %NormDerivee = (DeriveY.^2+DeriveX.^2).^0.5;
            %figure,imshow(NormDerivee,[0,10]),colormap(flipud(gray(256)),colorbar)
         
            %Calcul des fonctions d'interêts multi-échelle
            D1 = Detecteur(DeriveX,DeriveY,sigma_c1);
            D2 = Detecteur(DeriveX,DeriveY,sigma_c2);
    
            %Combinaison des fonctions d'interêts multi-échelle
            D = min(D1.*abs(D2),abs(D1).*D2);
            %figure,imshow(D,[-500,500]),colormap(flipud(gray(256)),colorbar)
    
            xp=zeros(1,nbPoints);%Vecteur abscisse des points de la frame i 
            yp=zeros(1,nbPoints);%Vecteur ordonnées des points de la frame i
            for p=1:nbPoints
                %Pour les deux premières frame l'estimation dynamique n'est
                %pas possible
                if(i==1 || i==2)
                    xEst=floor(xInit(p));
                    yEst=floor(yInit(p));
                else
                    %Estimation du point approximé en fonction des
                    %positions précédentes
                    xEst=EstimationDynamique(X(i-1,p),X(i-2,p));
                    yEst=EstimationDynamique(Y(i-1,p),Y(i-2,p));
                end
                
                %Récupération du maximum du voisinage des points approximatifs
                [xExa,yExa]=CoinsExacts(W,xEst,yEst,D,nbPoints,videoInfos);
    
                %Ajout des points au vecteurs des points de la frame i
                xp(p)=xExa;
                yp(p)=yExa;
            end
    
            %Ajout du vecteur aux matrices coordonnées
            X(i,:)=xp;
            Y(i,:)=yp;
    end
end

function filtre = FiltreG(x,sigma)
%Création d'un filtre moyenneur ou dérivateur gaussien 

    %Troncature du filtre gaussien continue suivant la règles des
    %3 sigmas
    N = 3*sigma;
    [X,Y]=meshgrid(-N:N);

    %Initialisation du filtre gaussien échantillioné
    G=(-1/(2*pi*sigma^2))*exp(-(X.^2+Y.^2)/(2*sigma^2));
    
    %Filtre dérivateur suivant X
    if x == 'X'
        filtreG = X.*G;
    %Filtre dérivateur suivant Y
    elseif x == 'Y'
        filtreG = Y.*G;
    %Filtre moyenneur
    else
        filtreG = G;
    end
    
    filtre = filtreG;
end

function D = Detecteur(DeriveX,DeriveY,sigma)
%Calcul de l'ensemble des covariances de la matrice A du détecteur de Harris
%(par moyennage gaussien) 

    %Création du filtre moyenneur gaussien
    G=FiltreG('G',sigma);

    %Calcul des termes de covariance
    Cxx=conv2((DeriveX.*DeriveX),G,'same');
    %figure, imshow(Cxx,[-1,1]), colorbar
    Cxy=conv2((DeriveX.*DeriveY),G,'same');
    %figure, imshow(Cxy,[-1,1]), colorbar
    Cyy=conv2((DeriveY.*DeriveY),G,'same');
    %figure, imshow(Cyy,[-1,1]), colorbar
    
    %Calcul de la fonction d'intérêt du détecteur de Harris 
    D=Cxx.*Cyy-Cxy.*Cxy-0.05*(Cxx+Cyy).^2;
end

function newX = EstimationDynamique(x1,x2)
%Estimation de la position du point suivant en fonction de la moyenne
%du déplacment des points précédents

    newX = x1 + floor((x1-x2)/2);
end

function [xExa,yExa] = CoinsExacts(W,xEst,yEst,Detecteur,nbPoints, videoInfos)
%Retourne la position exacte équivalent à un maximum local du détecteur 
%dans un voisinage des coins dont on a estimé la position

    for p=1:nbPoints
        %Coordonnées des coins de la fenêtre de recherche de maximum
        leftX=xEst-floor(W/2);
        rightX=xEst+floor(W/2);
        topY=yEst+floor(W/2);
        bottomY=yEst-floor(W/2);

        %Evite de sortir de l'image si la fenêtre de recherche est 
        %trop proche d'un bord
        if(leftX<1)
            leftX=1;
        end
        if(rightX>videoInfos.height)
            rightX=videoInfos.height;
        end
        if(topY<1)
            topY=1;
        end
        if(bottomY>videoInfos.width)
            bottomY=videoInfos.width;
        end

        %Récupération du voisinage du détecteur à considérer
        voisinageCoinij = Detecteur(leftX:rightX,bottomY:topY);

        %Récupération du max de chacune des colonnes
        [~,XCoin]=max(voisinageCoinij);
        %Récupération du max des max des colonnes
        [~,YCoin]=max(max(voisinageCoinij));

        %Transformation des coordonnées dans la fenêtre de recherche
        %en coordonnées dans l'image
        xExa=xEst-floor(W/2)+XCoin(YCoin);
        yExa=yEst-floor(W/2)+YCoin;
    end
end

%---- Test de notre détecteur ----%

function TestUnitaireTracking(X,Y,videoInfos)
%Fonction de visualisation des points suivis par l'intermédiare avec des
%petits carrés rouge les représentant. Méthode graphique de vérification des
%résultats de notre détecteur de Harris

    %Caractéristique du carré à afficher pour visualiser les coins suivis
    couleur =[255 0 0];
    taille = 5;

    %Nombre de coins suivis
    sizeX = size(X);

    figure,
    for i = 1 : videoInfos.NumFrames
        %Récupération de la l'image d'index "i" de la vidéo
        frame = read(videoInfos,i);
        %Création d'un carré rouge au niveau des coins suivis
        for j = 1:sizeX(2)
            for k=-taille:taille
                for l=-taille:taille
                    frame(X(i,j)+k,Y(i,j)+l,:) = couleur;
                end
            end
        end
        %Affichage de l'image modifié
        imshow(frame)
    end 
end


