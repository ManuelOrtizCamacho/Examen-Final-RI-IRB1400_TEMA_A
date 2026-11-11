%% ================================================================
%  EXAMEN FINAL - ROBÓTICA INDUSTRIAL
%  Análisis de velocidad: Jacobiano y movimiento en línea recta
%  Robot: ABB IRB 1400 (6 GDL)
%  Archivo de entrega: AnlVel_NNAA.m   (NN = nombre, AA = apellido)
%  ----------------------------------------------------------------
%  Reglas: solo código propio. NO se permite ninguna toolbox
%  (Robotics System Toolbox, Peter Corke, Symbolic, etc.).
%  Unidades: longitudes en mm, ángulos en grados para mostrar.
%% ================================================================
clc; clear; close all;

%% 0. DATOS DEL ESTUDIANTE  (no modificar la lógica)
carne  = '000000000';            % <-- escriba su carné completo
nombre = 'Nombre Apellido';      % <-- escriba su nombre
fprintf('Carné: %s | %s | %s\n', carne, nombre, datestr(now));

dg = carne(end-3:end) - '0';     % últimos 4 dígitos [d1 d2 d3 d4]
d1 = dg(1); d2 = dg(2); d3 = dg(3); d4 = dg(4);

% Configuración inicial q0 (grados)
q0_deg = [-45+10*d1, 5+2*d2, -15+3*d3, -40+10*d4, 35+3*d2, -45+10*d3];
q0     = deg2rad(q0_deg(:));

% Desplazamiento en línea recta del efector (mm)
s  = 2.5;
dp = s*[-(40+8*d4); -100+20*d1; -50+10*d3];

T_total = 2;      % duración del movimiento (s)
dt      = 0.01;   % paso de integración de Euler (s)

%% 1. PARÁMETROS DH  (0.4 Pts)
%  Columnas: [theta_offset(°)  d(mm)  a(mm)  alpha(°)]
%  theta_i = q_i + theta_offset_i
% TODO: complete la tabla DH a partir del plano (6 filas x 4 columnas)
DH = zeros(6,4);

Ts = fk(q0, DH);          % Ts{i+1} = 0_T_i ,  Ts{1} = eye(4)
T06 = Ts{7};
p0  = T06(1:3,4);
Pd  = p0 + dp;            % punto objetivo

disp('0_6T(q0) ='); disp(T06);
fprintf('Posición inicial p0 = [%.2f  %.2f  %.2f] mm\n', p0);
fprintf('Punto objetivo   Pd = [%.2f  %.2f  %.2f] mm\n', Pd);

figure('Name','Robot en q0'); dibujar_robot(Ts); title('Robot en q_0');

%% 2. JACOBIANO GEOMÉTRICO  (0.8 Pts)  -> ver función jacobiano() al final
J0 = jacobiano(q0, DH);
disp('J(q0) ='); disp(J0);

%% 3. VERIFICACIÓN NUMÉRICA DEL JACOBIANO (parte lineal)  (0.3 Pts)
h  = 1e-6;
Jn = zeros(3,6);
for i = 1:6
    dq = zeros(6,1);  dq(i) = h;
    Ti = fk(q0 + dq, DH);
    % TODO: columna i del Jacobiano lineal
    % Jn(:,i) = ...
end
error_max = max(max(abs(J0(1:3,:) - Jn)));
fprintf('Error máximo J analítico vs numérico = %.3e mm/rad\n', error_max);

%% 4. MOVIMIENTO EN LÍNEA RECTA CON EULER  (0.7 Pts)
t = 0:dt:T_total;   N = numel(t);
Q  = zeros(6,N);  Qd   = zeros(6,N);
P  = zeros(3,N);  Pdes = zeros(3,N);
q  = q0;
% TODO: velocidad lineal constante v y vector de velocidad cartesiana xd (6x1)
% v  = ...
% xd = ...
for k = 1:N
    Tk = fk(q, DH);
    P(:,k)    = Tk{7}(1:3,4);
    Pdes(:,k) = p0 + v*t(k);
    % TODO: Jacobiano en q, velocidad articular qd = J^-1 xd,
    %       guardar Q(:,k) y Qd(:,k), y actualizar q con Euler
end
err_final = norm(P(:,end) - Pd);
fprintf('Posición final obtenida = [%.2f  %.2f  %.2f] mm\n', P(:,end));
fprintf('Error final respecto a Pd = %.3f mm\n', err_final);

figure('Name','Trayectoria'); hold on;
dibujar_robot(fk(q0,DH)); dibujar_robot(fk(Q(:,end),DH));
plot3(Pdes(1,:),Pdes(2,:),Pdes(3,:),'k--','LineWidth',2);
plot3(P(1,:),P(2,:),P(3,:),'r','LineWidth',1.5);
title('Trayectoria deseada (--) vs obtenida (rojo)');

%% 5. LÍMITES ARTICULARES Y DE VELOCIDAD  (0.3 Pts)
% TODO: límites de la hoja de datos de su robot
qmin = zeros(1,6);   % grados
qmax = zeros(1,6);   % grados
vmax = zeros(1,6);   % grados/s
Qdeg  = rad2deg(Q);   Qddeg = rad2deg(Qd);
figure('Name','Articulaciones');
subplot(2,1,1); plot(t,Qdeg');  grid on; ylabel('q (°)');
subplot(2,1,2); plot(t,Qddeg'); grid on; ylabel('dq/dt (°/s)'); xlabel('t (s)');
legend('q1','q2','q3','q4','q5','q6');
fuera_rango = any(Qdeg < qmin' | Qdeg > qmax', 2)';
vel_max     = max(abs(Qddeg), [], 2)';
fuera_vel   = vel_max > vmax;
fprintf('Velocidad máxima por eje (°/s): %s\n', mat2str(round(vel_max*100)/100));
fprintf('Eje fuera de rango: %s\n', mat2str(fuera_rango));
fprintf('Eje supera velocidad: %s\n', mat2str(fuera_vel));
% CONCLUSIÓN (escriba aquí su análisis):
%

%% ======================= FUNCIONES ==============================
function A = dh(theta, d, a, alpha)
% Matriz homogénea DH estándar (ángulos en rad, longitudes en mm)
% TODO: matriz de transformación homogénea DH estándar (4x4)
A = eye(4);
end

function Ts = fk(q, DH)
% Cinemática directa: Ts{i+1} = 0_T_i
Ts = cell(1,7);  Ts{1} = eye(4);
for i = 1:6
    A = dh(q(i) + deg2rad(DH(i,1)), DH(i,2), DH(i,3), deg2rad(DH(i,4)));
    Ts{i+1} = Ts{i} * A;
end
end

function J = jacobiano(q, DH)
% Jacobiano geométrico 6x6 (articulaciones rotacionales)
Ts = fk(q, DH);
pn = Ts{7}(1:3,4);
J  = zeros(6,6);
for i = 1:6
    % TODO: columna i usando z_{i-1} y p_{i-1} (están en Ts{i})
end
end

function dibujar_robot(Ts)
% Dibuja eslabones (líneas entre orígenes) y sistemas coordenados
hold on; grid on; axis equal; view(135,25);
O = cell2mat(cellfun(@(T) T(1:3,4), Ts, 'UniformOutput', false));
plot3(O(1,:),O(2,:),O(3,:),'-o','LineWidth',3,'Color',[0.95 0.45 0.1],'MarkerFaceColor','k');
L = 0.08*max(1, max(abs(O(:))));
for i = 1:numel(Ts)
    o = Ts{i}(1:3,4); R = Ts{i}(1:3,1:3);
    quiver3(o(1),o(2),o(3),R(1,1)*L,R(2,1)*L,R(3,1)*L,0,'r','LineWidth',1.2);
    quiver3(o(1),o(2),o(3),R(1,2)*L,R(2,2)*L,R(3,2)*L,0,'g','LineWidth',1.2);
    quiver3(o(1),o(2),o(3),R(1,3)*L,R(2,3)*L,R(3,3)*L,0,'b','LineWidth',1.2);
end
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
end
