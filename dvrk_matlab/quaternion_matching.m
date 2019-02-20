%% *************************** Analitic REGISTRATION *****************************
% Funzione che viene utilizzata nei processi di registrazione. Viene
% classificato come metodo diretto (Horn method).
%
% NOTA: ? possibile utilizzare questo metodo se i due sistemi di punti sono
% corrispondenti. Ovvero n_pre=n_intra e il primo punto_pre corrisponde al
% primo punto_intra
%
%   Input (espressi sia in cc omogenee che normali) 3/4xn: 
%           - Pa: matrice di punti pre-operative
%           - Pb: matrice di punti intra-operative
%   Outpu:
%           - T: (tensore)matrice di trasformazione tra pre-operative a
%           intra-operative.
%
%   Per le fasi dell'algoritmo aprire testo della funzione.
%   ********************************************************************

% Passaggi algoritmo:
% 1. Viene calcolata la media dei due insieme di punti.
% 2. Vengono computati i punti sottraendo loro la media.
% 3. Viene calcolata la matrice H come sommatoria di tutte le coordinate.
% 4. Viene calcolata la matrice G come da teoria
% 5. Attraverso SVD(G)=Q che corrisponde al quaternione che continene
% autovettore massimo
% 6. Viene determinata la matrice T.


function [T] = quaternion_matching(Pa,Pb)

% Pa, Pb must be homogeneous coordinates with dimension 3/4 x n, where n is
% the number of points

Pa = Pa(1:3,:);   % Remove 1s
Pb = Pb(1:3,:);

dims = size(Pa);
n_points = dims(2);  % Number of points

Ba = mean(Pa,2);    % Center of mass
Bb = mean(Pb,2);

% Cross correlation function
cross_matrix = zeros(dims(1));
for i = 1 : n_points
    cross_matrix = cross_matrix + ((Pa(:,i) - Ba) * (Pb(:,i) - Bb)');
end
cross_matrix = 1 / n_points * cross_matrix;

% Calculate the parameters for the quaternion extraction
A = cross_matrix - cross_matrix';
D = [A(2,3) A(3,1) A(1,2)]';
Q = [trace(cross_matrix), D'; D, (cross_matrix + cross_matrix' - trace(cross_matrix) * eye(3))];

% Eigenval/Eigenvect extraction
[E_vec,E_val] = eig(Q);

% Find the maximum eigenval and its position
[max_v max_p] = max(diag(E_val));

% Calculates the quaternion NON MI ? CHIARO COSA SUCCEDE QUI?? COME CREA IL
% QUATERNIONE
quat = Quaternion(E_vec(:,max_p));

% Extract the rotation matrix COSA SONO QUESTE COSE CON IL PUNTO?
%T = quat.T;
T = quat.s;


% Calculates the translation component DOVE HO DEFINITO LA R!?!?!!?
%T(1:3,4) = (Bb - (quat.R) * Ba)';
T(1:3,4) = (Bb - (quat.v) * Ba)';

