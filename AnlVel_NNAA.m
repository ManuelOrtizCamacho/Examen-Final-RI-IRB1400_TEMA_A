% Examen Final - Robótica Industrial - Tema A: ABB IRB 1400
%% Profesor_Manuel_Ortiz_Camacho
% Análisis de velocidad: Jacobiano y movimiento en línea recta
% Archivo de entrega: AnlVel_NNAA.m  -  Solo código propio, sin toolboxes.
% Unidades: mm y rad (se muestran grados donde se indica).

clear all
clc
close all

%% 0. Datos del estudiante (no modificar la lógica)
carne  = '000000000';            % <-- escriba su carné completo
nombre = 'Nombre Apellido';      % <-- escriba su nombre
fprintf('Carné: %s | %s | %s\n', carne, nombre, datestr(now));

dg = carne(end-3:end) - '0';     % últimos 4 dígitos [d1 d2 d3 d4]
d1 = dg(1); d2 = dg(2); d3 = dg(3); d4 = dg(4);

q0_deg = [-45+10*d1, 5+2*d2, -15+3*d3, -40+10*d4, 35+3*d2, -45+10*d3];
q0 = deg2rad(q0_deg(:));                            % configuración inicial
dp = 2.5*[-(40+8*d4); -100+20*d1; -50+10*d3];        % desplazamiento recto (mm)

T_total = 2;      % duración del movimiento (s)
dt      = 0.01;   % paso de Euler (s)

%% 1. Direct Kinematic Model  (0.4 Pts)
% DH Matrix -> se completa en la función DH_robot(q) al final del archivo
% Columnas: [a_ij  alpha_ij  s_i  theta_i]   (mm, rad)

DH = DH_robot(q0)
T  = cinematica_directa(DH);
frame = size(DH,1);

T0n = T(:,:,frame)              % 0_nT en q0
O0  = T0n(1:3,4);               % posición inicial del efector
Pd  = O0 + dp;                  % punto objetivo
fprintf('Posición inicial O0 = [%.2f  %.2f  %.2f] mm\n', O0);
fprintf('Punto objetivo   Pd = [%.2f  %.2f  %.2f] mm\n', Pd);

%% Graph
hf = figure(1);
set(hf,'position',[445 275 563 628])
graficar_robot(T, 100, [-400 1400 -800 1000 0 1600]);
title('Robot en q_0')

%% 2. Jacobiano de velocidad  (0.8 Pts) -> ver función jacobiano(q) al final
%  Ec. (27):  J_vi = z_i x (O_n - O_i)     J_wi = z_i
J0 = jacobiano(q0)

%% 3. Verificación del Jacobiano lineal con la ec. (22)  (0.3 Pts)
%  J_vi = d(0_nO)/dq_i  -> derivada numérica moviendo q_i un valor pequeño h
h  = 1e-6;
Jn = zeros(3,6);
for i = 1:6
    dq = zeros(6,1);  dq(i) = h;
    Ti = cinematica_directa(DH_robot(q0 + dq));
    % TODO: columna i del Jacobiano lineal con la ec. (22)
    % Jn(:,i) = ...
end
error_max = max(max(abs(J0(1:3,:) - Jn)));
fprintf('Error máximo ec.(27) vs ec.(22) = %.3e mm/rad\n', error_max);

%% 4. Movimiento en línea recta con Euler  (0.7 Pts)
t = 0:dt:T_total;   N = numel(t);
Q  = zeros(6,N);  Qd   = zeros(6,N);
P  = zeros(3,N);  Pdes = zeros(3,N);
q  = q0;
% TODO: velocidad lineal constante v (3x1) y velocidad cartesiana xd (6x1)
% v  = ...
% xd = ...
for k = 1:N
    Tk = cinematica_directa(DH_robot(q));
    P(:,k)    = Tk(1:3,4,frame);          % posición obtenida
    Pdes(:,k) = O0 + v*t(k);              % posición deseada (recta)
    % TODO: Jacobiano en q, velocidad articular qd = J^-1 * xd,
    %       guardar Q(:,k) y Qd(:,k), y actualizar q con Euler
end
err_final = norm(P(:,end) - Pd);
fprintf('Posición final obtenida = [%.2f  %.2f  %.2f] mm\n', P(:,end));
fprintf('Error final respecto a Pd = %.3f mm\n', err_final);

hf2 = figure(2);
set(hf2,'position',[1010 275 563 628])
graficar_robot(cinematica_directa(DH_robot(Q(:,end))), 100, [-400 1400 -800 1000 0 1600]);
plot3(Pdes(1,:),Pdes(2,:),Pdes(3,:),'k--','linewidth',2)
plot3(P(1,:),P(2,:),P(3,:),'m','linewidth',1.5)
title('Trayectoria deseada (--) vs obtenida (magenta)')

%% 5. Límites articulares y de velocidad  (0.3 Pts)
% TODO: límites de la hoja de datos de su robot
qmin = zeros(1,6);   % grados
qmax = zeros(1,6);   % grados
vmax = zeros(1,6);   % grados/s
Qdeg = rad2deg(Q);   Qddeg = rad2deg(Qd);
figure(3)
subplot(2,1,1); plot(t,Qdeg');  grid on; ylabel('q [°]')
subplot(2,1,2); plot(t,Qddeg'); grid on; ylabel('dq/dt [°/s]'); xlabel('t [s]')
legend('q1','q2','q3','q4','q5','q6')
fuera_rango = any(Qdeg < qmin' | Qdeg > qmax', 2)';
vel_max     = max(abs(Qddeg), [], 2)';
fuera_vel   = vel_max > vmax;
fprintf('Velocidad máxima por eje [°/s]: %s\n', mat2str(round(vel_max*100)/100));
fprintf('Eje fuera de rango:   %s\n', mat2str(fuera_rango));
fprintf('Eje supera velocidad: %s\n', mat2str(fuera_vel));
% CONCLUSIÓN (escriba aquí su análisis):
%


%% ===================== FUNCIONES =====================

%% DH Matrix del robot en función de q (rad)
function DH = DH_robot(q)
%  Columnas: [a_ij  alpha_ij  s_i  theta_i]  (mm, rad)
%  Filas 1-6: articulaciones. Fila 7: brida (efector final).
    % TODO: complete la matriz DH con las medidas del plano
    DH = zeros(7,4);
end

%% Cinemática directa: T(:,:,i) = 0_iT
function T = cinematica_directa(DH)
frame = size(DH,1);
for i = 1:frame
    T_ij(:,:,i) = T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
end
T(:,:,1) = T_ij(:,:,1);
for i = 1:frame-1
    T(:,:,i+1) = eye(4);   % TODO: 0_(i+1)T a partir de 0_iT y de la matriz i+1
end
end

%% Jacobiano geométrico 6x6 - ec. (27)
function J = jacobiano(q)
T  = cinematica_directa(DH_robot(q));
On = T(1:3,4,end);          % origen del efector final (brida)
J  = zeros(6,6);
for i = 1:6
    % TODO: columna i con la ec. (27): J_vi = z_i x (O_n - O_i),  J_wi = z_i
    %       z_i y O_i se obtienen de T(:,:,i)
end
end

%% Gráfica de los sistemas coordenados y eslabones
function graficar_robot(T, L, ejes)
frame = size(T,3);
plot3([0 L],[0 0],[0 0],'r','linewidth',2.5); hold on
plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
axis equal
axis(ejes)
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]')
view(27,37); grid on

Mp = [0,0,0,1;  % origen
      L,0,0,1;  % eje x
      0,L,0,1;  % eje y
      0,0,L,1]; % eje z
Mp_T = zeros(4,4,frame);
for k = 1:frame
    for i = 1:4
        Mp_T(i,:,k) = (T(:,:,k)*Mp(i,:)')';
    end
end
O = [zeros(3,1), squeeze(T(1:3,4,:))];
plot3(O(1,:),O(2,:),O(3,:),'k-o','linewidth',3)            % eslabones
for k = 1:frame
    plot3([Mp_T(1,1,k) Mp_T(2,1,k)],[Mp_T(1,2,k) Mp_T(2,2,k)],[Mp_T(1,3,k) Mp_T(2,3,k)],'r','linewidth',2)
    plot3([Mp_T(1,1,k) Mp_T(3,1,k)],[Mp_T(1,2,k) Mp_T(3,2,k)],[Mp_T(1,3,k) Mp_T(3,3,k)],'g','linewidth',2)
    plot3([Mp_T(1,1,k) Mp_T(4,1,k)],[Mp_T(1,2,k) Mp_T(4,2,k)],[Mp_T(1,3,k) Mp_T(4,3,k)],'b','linewidth',2)
end
end

%% Transform tipo Denavit-Hartemberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)

T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end
