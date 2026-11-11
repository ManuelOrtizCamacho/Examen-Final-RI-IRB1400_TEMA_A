# Examen final — Robótica Industrial · Tema A: ABB IRB 1400

**Análisis de velocidad: Jacobiano de velocidad y movimiento en línea recta**

| | |
|---|---|
| Duración | **55 minutos** |
| Valor | **2.5 Pts** |
| Herramienta | MATLAB, **solo código propio** (no se permite ninguna toolbox: Robotics System Toolbox, Peter Corke, Symbolic, etc.) |
| Archivo base | [`AnlVel_NNAA.m`](AnlVel_NNAA.m) |

## Cómo empezar

1. Descargue este repositorio (**Code → Download ZIP**) o clónelo.
2. Renombre `AnlVel_NNAA.m` con sus iniciales (NN = nombre, AA = apellido).
3. En la sección 0 del archivo escriba **su carné completo** y su nombre. Sus valores de **q₀** y del desplazamiento se calculan automáticamente a partir de los **últimos 4 dígitos del carné**. No los cambie: cada estudiante tiene un problema distinto.
4. Complete todas las líneas marcadas con `TODO`.

## Plano del robot (medidas en mm)

![Plano ABB IRB 1400](plano_IRB1400.png)

> Use el plano para obtener las medidas del robot. No todas las cotas del plano son parámetros DH.

## Límites de la hoja de datos

| Eje | Rango articular | Velocidad máxima |
|---|---|---|
| 1 | −170° a +170° | 110 °/s |
| 2 | −70° a +70° | 110 °/s |
| 3 | −70° a +65° | 110 °/s |
| 4 | −150° a +150° | 280 °/s |
| 5 | −115° a +115° | 280 °/s |
| 6 | −300° a +300° | 280 °/s |

## Enunciado

En el ABB IRB 1400 (6 GDL), con la configuración articular **q₀** y el punto objetivo **P_d = O₀ + Δp** que genera su carné, determine:

1. **(0.4 Pts)** La matriz de parámetros DH (función `DH_robot(q)`, filas 1–6 para las articulaciones y fila 7 para la brida) y la matriz de transformación homogénea ⁰ₙT(q₀) (función `cinematica_directa`).
2. **(0.8 Pts)** El Jacobiano de velocidad **J(q)** de 6×6 (función `jacobiano`), construido columna por columna con la **ec. (27)**:
   J_vi = z_i × (O_n − O_i),  J_ωi = z_i
   donde z_i y O_i se obtienen de ⁰ᵢT.
3. **(0.3 Pts)** Verificar la parte lineal de J(q₀) con la **ec. (22)**, J_vi = ∂ ⁰ₙO / ∂q_i, calculando la derivada **numéricamente**: mover cada articulación un valor pequeño h = 1e‑6 rad y medir cuánto cambia la posición del efector. Reportar el error máximo entre ambos.
4. **(0.7 Pts)** Mover el efector final en **línea recta** desde O₀ hasta P_d en T = 2 s, **manteniendo la orientación**, usando **q̇ = J⁻¹ẋ** e integrando con Euler (δt = 0.01 s). Graficar la trayectoria deseada frente a la obtenida sobre el robot.
5. **(0.3 Pts)** Graficar q(t) y q̇(t), completar los límites de la hoja de datos y concluir si algún eje se sale de su rango o supera su velocidad máxima.

## Entrega

Suba a Moodle una carpeta comprimida con:

1. **`AnlVel_NNAA.png`**: captura de pantalla completa donde se evidencie:
   - La interfaz de MATLAB con el código solución (porción más relevante).
   - La **Command Window** con la salida del programa: la línea con su carné, nombre, fecha y hora, la posición inicial **O₀**, la posición final obtenida, el error final, el error ec.(27) vs ec.(22) y la velocidad máxima por eje.
   - La sesión activa de Moodle donde se evidencie el examen y el usuario.
2. **`AnlVel_NNAA.m`**: su archivo completo. Debe ejecutarse sin errores y reproducir los valores de la captura.
