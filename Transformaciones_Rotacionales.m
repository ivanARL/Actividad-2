%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 ROBOT ANTROPOMÓRFICO 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Limpieza de pantalla
clearvars % Cambiado de clear all a clearvars para mejorar rendimiento
close all
clc
%Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) th4(t) th5(t) t l1 l2 l3 l4 l5
%Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP=[0 0 0 0 0];
%Creamos el vector de coordenadas articulares
Q= [th1, th2, th3, th4, th5];
disp('Coordenadas generalizadas (Antropomórfico)');
pretty (Q);
%Creamos el vector de velocidades generalizadas
Qp= diff(Q, t);
disp('Velocidades generalizadas');
pretty (Qp);
%Número de grado de libertad del robot
GDL= size(RP,2);
GDL_str= num2str(GDL);

%Junta 1 
%Posición y Matriz de rotación de la junta 1
P(:,:,1)= [0; 0; l1];
R(:,:,1)= [0, -sin(th1), cos(th1);
           0,  cos(th1), sin(th1);
          -1,         0,        0];
%Junta 2
%Posición y Matriz de rotación de la junta 2
P(:,:,2)= [-l2*sin(th2); -l2*cos(th2); 0];
R(:,:,2)= [1, 0, 0;
           0, 1, 0;
           0, 0, 1];
%Junta 3
%Posición y Matriz de rotación de la junta 3
P(:,:,3)= [-l3*sin(th3); l3*cos(th3); 0];
R(:,:,3)= [cos(th3), 0, sin(th3);
           sin(th3), 0, -cos(th3);
           0,        1,        0];
%Junta 4
%Posición y Matriz de rotación de la junta 4
P(:,:,4)= [0; 0; -l4];
R(:,:,4)= [cos(th4), 0,  sin(th4);
           sin(th4), 0, -cos(th4);
           0,        1,        0];
%Junta 5
%Posición y Matriz de rotación de la junta 5
P(:,:,5)= [-l5*sin(th5); -l5*cos(th5); 0];
R(:,:,5)= [cos(th5), 0, -sin(th5);
           sin(th5), 0,  cos(th5);
           0,       -1,         0];

%Creamos un vector de ceros
Vector_Zeros= zeros(1, 3);
%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 
%Inicializamos las INVERSAS de las matrices de rotación vistas desde el marco de referencia inercial
RO_inv(:,:,GDL)= R(:,:,GDL); 

for i = 1:GDL
    i_str= num2str(i);
    %Locales
    disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    pretty (A(:,:,i));
   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
    disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
    pretty(T(:,:,i))
    RO(:,:,i)= T(1:3,1:3,i);
    RO_inv(:,:,i)= transpose(RO(:,:,i));
    PO(:,:,i)= T(1:3,4,i);
end

%Calculamos el jacobiano lineal de forma diferencial
disp('Jacobiano lineal obtenido de forma diferencial');
% NOTA: Se cambia functionalDerivative por jacobian para evitar el error en MATLAB
jv_d = simplify(jacobian(PO(:,:,GDL), Q));
pretty(jv_d);

%Calculamos el jacobiano lineal de forma analítica
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);

for k= 1:GDL
    if RP(k)==0 %Casos: articulación rotacional
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));
            Jw_a(:,k)=[0,0,1];
        end
        
     elseif RP(k)==1 %Casos: articulación prismática
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    
 
Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);
disp('Jacobiano lineal obtenido de forma analítica');
pretty (Jv_a);
disp('Jacobiano ángular obtenido de forma analítica');
pretty (Jw_a);

disp('Velocidad lineal obtenida mediante el Jacobiano lineal');
V=simplify (Jv_a*Qp');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano angular');
W=simplify (Jw_a*Qp');
pretty(W);

disp('-----------------------------------------------------------');
disp('Presiona ENTER en la ventana de comandos para continuar con el Robot Industrial...');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   ROBOT INDUSTRIAL 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Limpieza de pantalla
clearvars 
clc
%Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) th4(t) th5(t) t l1 l2 l3 l4 l5
%Configuración del robot
RP=[0 0 0 0 0];
%Creamos el vector de coordenadas articulares
Q= [th1, th2, th3, th4, th5];
disp('Coordenadas generalizadas (Industrial)');
pretty (Q);
%Creamos el vector de velocidades generalizadas
Qp= diff(Q, t);
disp('Velocidades generalizadas');
pretty (Qp);
%Número de grado de libertad del robot
GDL= size(RP,2);
GDL_str= num2str(GDL);

%Junta 1 
P(:,:,1) = [0; 0; l1];
R(:,:,1) = [0, -sin(th1), -cos(th1);
            0,  cos(th1), -sin(th1);
            1,         0,         0];
%Junta 2
P(:,:,2) = [-l2*sin(th2); l2*cos(th2); 0];
R(:,:,2) = [-cos(th2), -sin(th2),  0;
            -sin(th2),  cos(th2),  0;
             0,         0,        -1];
%Junta 3
P(:,:,3) = [-l3*sin(th3); l3*cos(th3); 0];
R(:,:,3) = [-cos(th3), 0, -sin(th3);
             sin(th3), 0, -cos(th3);
             0,       -1,         0];
%Junta 4
P(:,:,4) = [0; 0; l4];
R(:,:,4) = [-cos(th4), 0,  sin(th4);
             sin(th4), 0, -cos(th4);
             0,        1,         0];
%Junta 5
P(:,:,5) = [-l5*sin(th5); l5*cos(th5); 0]; 
R(:,:,5) = [-cos(th5), 0,  sin(th5);
             sin(th5), 0, -cos(th5);
             0,        1,         0];

%Creamos un vector de ceros
Vector_Zeros= zeros(1, 3);
%Inicializamos las matrices
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
PO(:,:,GDL)= P(:,:,GDL); 
RO(:,:,GDL)= R(:,:,GDL); 
RO_inv(:,:,GDL)= R(:,:,GDL); 

for i = 1:GDL
    i_str= num2str(i);
    %Locales
    disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    pretty (A(:,:,i));
   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
    disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
    pretty(T(:,:,i))
    RO(:,:,i)= T(1:3,1:3,i);
    RO_inv(:,:,i)= transpose(RO(:,:,i));
    PO(:,:,i)= T(1:3,4,i);
end

%Calculamos el jacobiano lineal de forma diferencial
disp('Jacobiano lineal obtenido de forma diferencial');
jv_d = simplify(jacobian(PO(:,:,GDL), Q));
pretty(jv_d);

%Calculamos el jacobiano lineal de forma analítica
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);

for k= 1:GDL
    if RP(k)==0 
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));
            Jw_a(:,k)=[0,0,1];
        end
        
     elseif RP(k)==1 
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    
 
Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);
disp('Jacobiano lineal obtenido de forma analítica');
pretty (Jv_a);
disp('Jacobiano ángular obtenido de forma analítica');
pretty (Jw_a);

disp('Velocidad lineal obtenida mediante el Jacobiano lineal');
V=simplify (Jv_a*Qp');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano angular');
W=simplify (Jw_a*Qp');
pretty(W);