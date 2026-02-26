%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Archivo sugerido: Actividad_2_Transformaciones.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 ROBOT ANTROPOMÓRFICO (5 TRANSFORMACIONES)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars
close all
clc

% SECCIÓN 1: Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) th4(t) th5(t) t l1 l2 l3 l4 l5

% SECCIÓN 2: Configuración del robot (0 = rotacional)
RP = [0 0 0 0 0];

% SECCIÓN 3 y 4: Coordenadas y Velocidades generalizadas
Q = [th1, th2, th3, th4, th5];
disp('Coordenadas generalizadas (Antropomórfico)');
pretty(Q);

Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

% SECCIÓN 5: Número de grados de libertad del robot
GDL = size(RP,2);

% SECCIÓN 6: Definición geométrica de las juntas
P(:,:,1) = [0; 0; l1];
R(:,:,1) = [0, -sin(th1), cos(th1);
            0,  cos(th1), sin(th1);
           -1,         0,        0];

P(:,:,2) = [-l2*sin(th2); -l2*cos(th2); 0];
R(:,:,2) = [1, 0, 0;
            0, 1, 0;
            0, 0, 1];

P(:,:,3) = [-l3*sin(th3); l3*cos(th3); 0];
R(:,:,3) = [cos(th3), 0, sin(th3);
            sin(th3), 0, -cos(th3);
            0,        1,        0];

P(:,:,4) = [0; 0; -l4];
R(:,:,4) = [cos(th4), 0, sin(th4);
            sin(th4), 0, -cos(th4);
            0,        1,        0];

P(:,:,5) = [-l5*sin(th5); -l5*cos(th5); 0];
R(:,:,5) = [cos(th5), 0, -sin(th5);
            sin(th5), 0,  cos(th5);
            0,       -1,         0];

% SECCIÓN 7: Inicialización de matrices
Vector_Zeros = zeros(1, 3);
A = sym(zeros(4, 4, GDL));
T = sym(zeros(4, 4, GDL));
PO = sym(zeros(3, 1, GDL)); 
RO = sym(zeros(3, 3, GDL)); 

% SECCIÓN 8: Cálculo de matrices de transformación
for i = 1:GDL
    i_str = num2str(i);
    disp(['Matriz de Transformación local A', i_str]);
    A(:,:,i) = simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    pretty(A(:,:,i));
    
    if i == 1
       T(:,:,i) = A(:,:,i);
    else
       T(:,:,i) = simplify(T(:,:,i-1) * A(:,:,i));
    end
    
    disp(['Matriz de Transformación global T', i_str]);
    pretty(T(:,:,i))
    
    RO(:,:,i) = T(1:3, 1:3, i);
    PO(:,:,i) = T(1:3, 4, i);
end

% SECCIÓN 9: Cálculo del Jacobiano lineal (diferencial)
disp('Jacobiano lineal obtenido de forma diferencial');
jv_d = sym(zeros(3, GDL));
for i = 1:3
    for j = 1:GDL
        jv_d(i,j) = functionalDerivative(PO(i,1,GDL), Q(j));
    end
end
jv_d = simplify(jv_d);
pretty(jv_d);

% SECCIÓN 10: Jacobiano analítico y cálculo de velocidades
Jv_a = sym(zeros(3, GDL));
Jw_a = sym(zeros(3, GDL));

for k= 1:GDL
    if k == 1
        Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));
        Jw_a(:,k) = [0;0;1];
    else
        Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
        Jw_a(:,k) = RO(:,3,k-1);
    end
end    

Jv_a = simplify(Jv_a);
Jw_a = simplify(Jw_a);
disp('Jacobiano lineal (Analítico)');
pretty(Jv_a);
disp('Jacobiano angular (Analítico)');
pretty(Jw_a);

disp('Velocidad lineal del efector final');
V = simplify(Jv_a * Qp');
pretty(V);
disp('Velocidad angular del efector final');
W = simplify(Jw_a * Qp');
pretty(W);


disp('-----------------------------------------------------------');
disp('Presiona ENTER en la ventana de comandos para continuar con el Robot Industrial...');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   ROBOT INDUSTRIAL (5 TRANSFORMACIONES)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars
clc

% SECCIÓN 1: Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) th4(t) th5(t) t l1 l2 l3 l4 l5

% SECCIÓN 2: Configuración del robot
RP = [0 0 0 0 0];

% SECCIÓN 3 y 4: Coordenadas y Velocidades generalizadas
Q = [th1, th2, th3, th4, th5];
disp('Coordenadas generalizadas (Industrial)');
pretty(Q);

Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

% SECCIÓN 5: Grados de libertad
GDL = size(RP,2);

% SECCIÓN 6: Definición geométrica de las juntas (Actualizado según PDF nuevo)
P(:,:,1) = [0; 0; l1];
R(:,:,1) = [0, -sin(th1), -cos(th1);
            0,  cos(th1), -sin(th1);
            1,         0,         0];

P(:,:,2) = [-l2*sin(th2); l2*cos(th2); 0];
R(:,:,2) = [-cos(th2), -sin(th2),  0;
            -sin(th2),  cos(th2),  0;
             0,         0,        -1];

P(:,:,3) = [-l3*sin(th3); l3*cos(th3); 0];
R(:,:,3) = [-cos(th3), 0, -sin(th3);
             sin(th3), 0, -cos(th3);
             0,       -1,         0];

P(:,:,4) = [0; 0; l4];
R(:,:,4) = [-cos(th4), 0,  sin(th4);
             sin(th4), 0, -cos(th4);
             0,        1,         0];

P(:,:,5) = [-l5*sin(th5); l5*cos(th5); 0]; % Corregido th4 a th5 lógicamente
R(:,:,5) = [-cos(th5), 0,  sin(th5);
             sin(th5), 0, -cos(th5);
             0,        1,         0];

% SECCIÓN 7: Inicialización de matrices
Vector_Zeros = zeros(1, 3);
A = sym(zeros(4, 4, GDL));
T = sym(zeros(4, 4, GDL));
PO = sym(zeros(3, 1, GDL)); 
RO = sym(zeros(3, 3, GDL)); 

% SECCIÓN 8: Cálculo de matrices de transformación
for i = 1:GDL
    i_str = num2str(i);
    disp(['Matriz de Transformación local A', i_str]);
    A(:,:,i) = simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    pretty(A(:,:,i));
    
    if i == 1
       T(:,:,i) = A(:,:,i);
    else
       T(:,:,i) = simplify(T(:,:,i-1) * A(:,:,i));
    end
    
    disp(['Matriz de Transformación global T', i_str]);
    pretty(T(:,:,i))
    
    RO(:,:,i) = T(1:3, 1:3, i);
    PO(:,:,i) = T(1:3, 4, i);
end

% SECCIÓN 9: Cálculo del Jacobiano lineal (diferencial)
disp('Jacobiano lineal obtenido de forma diferencial');
jv_d = sym(zeros(3, GDL));
for i = 1:3
    for j = 1:GDL
        jv_d(i,j) = functionalDerivative(PO(i,1,GDL), Q(j));
    end
end
jv_d = simplify(jv_d);
pretty(jv_d);

% SECCIÓN 10: Jacobiano analítico y cálculo de velocidades
Jv_a = sym(zeros(3, GDL));
Jw_a = sym(zeros(3, GDL));

for k= 1:GDL
    if k == 1
        Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));
        Jw_a(:,k) = [0;0;1];
    else
        Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
        Jw_a(:,k) = RO(:,3,k-1);
    end
end    

Jv_a = simplify(Jv_a);
Jw_a = simplify(Jw_a);
disp('Jacobiano lineal (Analítico)');
pretty(Jv_a);
disp('Jacobiano angular (Analítico)');
pretty(Jw_a);

disp('Velocidad lineal del efector final');
V = simplify(Jv_a * Qp');
pretty(V);
disp('Velocidad angular del efector final');
W = simplify(Jw_a * Qp');
pretty(W);