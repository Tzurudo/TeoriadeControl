# Tarea 2 – Ingeniería de Control Moderno

**Universidad Autónoma Chapingo**  
**Departamento de Ingeniería Mecánica Agrícola**  
**Ingeniería de Control Moderno**  
**Tarea No. 2**  

---

## 1. Espacio de estados y realimentación

### a) Representación en espacio de estados

Planta:  
$$
Y(s)/U(s) = \frac{10}{(s+1)(s+2)(s+3)}
$$

Variables de estado:  
- \( x_1 = y \)
- \( x_2 = \dot{x}_1 = \dot{y} \)
- \( x_3 = \dot{x}_2 = \ddot{y} \)

Forma canónica controlable:

\[
\begin{align*}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= x_3 \\
\dot{x}_3 &= -6x_1 - 11x_2 - 6x_3 + 10u \\
y &= x_1
\end{align*}
\]

Matrices:
\[
A = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 1 \\ -6 & -11 & -6 \end{bmatrix},\quad
B = \begin{bmatrix} 0 \\ 0 \\ 10 \end{bmatrix},\quad
C = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}
\]

---

### b) Controlabilidad

**Analítica:**  
La matriz de controlabilidad \( \mathcal{C} = [B \ AB \ A^2B] \) debe tener rango 3.

**MATLAB:**
```matlab
A = [0 1 0; 0 0 1; -6 -11 -6];
B = [0; 0; 10];
Co = ctrb(A, B);
rank_Co = rank(Co); % Debe ser 3
```

---

### c) Ganancia de retroalimentación \( K \)

Polos deseados: \( s = -2 \pm 3j,\, s = -10 \)

**MATLAB:**
```matlab
p = [-2+3j, -2-3j, -10];
K = place(A, B, p); % K = [k1 k2 k3]
```

---

### d) Simulación en lazo cerrado

**Condiciones iniciales propuestas (ejemplo):**
```matlab
x0 = [1; 0; 0]; % Modifica según lo requerido
sys_cl = ss(A-B*K, B, C, 0);
t = 0:0.01:5;
[Y, T, X] = initial(sys_cl, x0, t);

plot(T, Y), grid on
title('Respuesta en lazo cerrado');
xlabel('Tiempo [s]'); ylabel('Salida y(t)');
```

---

## 2. Uso de tf2ss en MATLAB

### a) Representación espacio de estados

```matlab
num = 10;
den = conv([1 1],[1 2]);
den = conv(den, [1 3]);
[A, B, C, D] = tf2ss(num, den);
```

---

### b) Controlabilidad

```matlab
Co = ctrb(A, B);
rank_Co = rank(Co); % Debe ser 3
```

---

### c) Matriz \( K \)

```matlab
p = [-2+3j, -2-3j, -10];
K = place(A, B, p);
```

---

### d) Simulación en lazo cerrado

```matlab
x0 = [1; 0; 0];
sys_cl = ss(A-B*K, B, C, 0);
t = 0:0.01:5;
[Y, T, X] = initial(sys_cl, x0, t);
plot(T, Y), grid on
title('Respuesta en lazo cerrado');
```

---

## 3. Sistema no estabilizable

\[
\dot{x} = \begin{bmatrix} 0 & 1 \\ 20 & 10 \end{bmatrix} x + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u
\]

No es estabilizable porque el par (A, B) no cumple criterios de controlabilidad para polos en el semiplano derecho. Puedes verificar con:

```matlab
A = [0 1; 20 10];
B = [0; 1];
Co = ctrb(A, B);
rank_Co = rank(Co); % Debe ser 2 (sí es controlable, pero los polos no pueden ser llevados al semiplano izquierdo)
eig(A) % Verifica los polos
```
Explicación: ambos polos tienen parte real positiva, pero la matriz B no permite colocar los polos en el semiplano izquierdo con retroalimentación de estados.

---

## 4. Sistema con A, B dadas

A = \(\begin{bmatrix} -6 & 5 & 1 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}\),  
B = \(\begin{bmatrix} 1 \\ 1 \\ 0 \end{bmatrix}\)

### a) Controlabilidad

```matlab
A = [-6 5 1; 1 0 0; 0 1 0];
B = [1; 1; 0];
Co = ctrb(A, B);
rank_Co = rank(Co); % Debe ser 3
```

### b) Matriz \( K \)

```matlab
p = [-2+4j, -2-4j, -10];
K = place(A, B, p);
```

### c) Simulación

```matlab
x0 = [1; 0; 0];
C = [1 0 0];
sys_cl = ss(A-B*K, B, C, 0);
t = 0:0.01:5;
[Y, T, X] = initial(sys_cl, x0, t);
plot(T, Y), grid on
```

---

## 5. Sistema con entrada de referencia

A = \(\begin{bmatrix} -6 & 5 & 0 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}\),  
B = \(\begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}\)  
C = \([0\ 0\ 1]\)

u(t) = r - (k1 x1 + k2 x2 + k3 x3)

### a) Controlabilidad

```matlab
A = [-6 5 0; 1 0 0; 0 1 0];
B = [1; 0; 0];
Co = ctrb(A, B);
rank_Co = rank(Co);
```

### b) Ganancias \( k_1, k_2, k_3 \)

```matlab
p = [-2+4j, -2-4j, -10];
K = place(A, B, p); % K = [k1 k2 k3]
```

### c) Respuesta a escalón

```matlab
C = [0 0 1];
sys_cl = ss(A-B*K, B, C, 0);
t = 0:0.01:5;
step(sys_cl, t)
title('Respuesta a escalón unitario');
```

---

## 6. Péndulo invertido

Ecuaciones linealizadas dadas. Parámetros: M = 2 kg, m = 0.5 kg, l = 1 m  
Variables:  
- \( x_1 = \theta \)
- \( x_2 = \dot{\theta} \)
- \( x_3 = x \)
- \( x_4 = \dot{x} \)

### a) Forma de estado

\[
\dot{x} = A x + B u
\]
Donde A y B deben deducirse usando las ecuaciones dadas y sustituyendo los parámetros.

### b) Controlabilidad

```matlab
% Define A y B usando tus ecuaciones y parámetros
Co = ctrb(A, B);
rank_Co = rank(Co); % Debe ser 4
```

### c) Ganancia \( K \)

```matlab
p = [-2+4j, -2-4j, -3, -4];
K = place(A, B, p);
```

### d) Simulación con condición inicial

```matlab
x0 = [0; 0; 0; 1];
C = eye(4); % O solo las salidas requeridas
sys_cl = ss(A-B*K, B, C, 0);
t = 0:0.01:10;
[Y, T, X] = initial(sys_cl, x0, t);
plot(T, Y(:,1)), hold on, plot(T, Y(:,3)), grid on
legend('\theta', 'x')
```

### e) Simulación con entrada escalón

```matlab
sys_cl = ss(A-B*K, B, eye(4), 0);
t = 0:0.01:10;
step(sys_cl, t)
```

---

## 7. Observabilidad y observador

A = \(\begin{bmatrix} 0 & 6 & 5 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}\), B, C como arriba.

### a) Observabilidad

```matlab
A = [0 6 5; 1 0 0; 0 1 0];
C = [0 0 1];
Ob = obsv(A, C);
rank_Ob = rank(Ob); % Debe ser 3
```

### b) Diseño de observador

```matlab
L = place(A', C', [-1, -2, -3])'; % Ganancias del observador
```

---

## 8 y 9. Sistemas según diagrama

**Nota:** Especificar el sistema según el diagrama (no incluido aquí).  
Sigue pasos similares a los anteriores para:  
- Estado, controlabilidad, observabilidad  
- Diseño de observador (L) y K  
- Simulación  
- Respuesta a escalón

---

## 10. Diseño de observador con requisitos de amortiguamiento y frecuencia

- ζ = 0.4, ω_n = 75
- Polos: \( s_{1,2} = -\zeta \omega_n \pm j \omega_n \sqrt{1-\zeta^2} \)
- Tercer polo: \( s_3 = -10\times|\text{Re}(s_1)| \)

```matlab
zeta = 0.4; wn = 75;
s1 = -zeta*wn + 1j*wn*sqrt(1-zeta^2);
s2 = -zeta*wn - 1j*wn*sqrt(1-zeta^2);
s3 = -10*abs(real(s1));
L = place(A', C', [s1 s2 s3])'; % Ajusta A y C según tu sistema
```

---

**Recuerda:**  
- Cambia condiciones iniciales y parámetros según lo que se pida en cada inciso.
- Documenta tus scripts y códigos.
- Si te piden simulaciones en MATLAB, usa `initial` para condiciones iniciales y `step` para respuesta a escalón.

---

¿Necesitas los scripts .m para cada ejercicio? ¿Quieres que te arme los archivos por separado?
