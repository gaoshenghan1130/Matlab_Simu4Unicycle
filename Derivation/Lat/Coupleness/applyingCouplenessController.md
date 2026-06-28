# Applying couplenss controller 

Now we get the controller definition, we can 

$$
\begin{bmatrix}
2m_L R^2 + m_b(R+h)^2 + m_w R^2 + I_b + I_w + I_{rod} + 2m_L r^2 & 2m_L R \\
2m_L R & 2m_L
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta} \\
\ddot r
\end{bmatrix}
= \begin{bmatrix}
2m_L gR\sin\theta + 2m_L gr\cos\theta + m_b g(R+h)\sin\theta + m_w gR\sin\theta - 4m_L r\dot r\dot\theta \\
2m_L g\sin\theta + F - F_r + 2m_L r\dot\theta^2
\end{bmatrix}
$$

Formulate in the form of $\dot{z} = f(z, u)$, we have:

$$
z = \begin{bmatrix} z_1 \\ z_2 \\ z_3 \\ z_4 \end{bmatrix} = \begin{bmatrix} \theta \\ r \\ \dot{\theta} \\ \dot{r} \end{bmatrix}, \quad u = F
$$

$$
M(z_2) = \begin{bmatrix} 
M_0 + 2m_L z_2^2 & 2m_L R \\ 
2m_L R & 2m_L 
\end{bmatrix}
$$

$$
H(z) = \begin{bmatrix} 
(2m_L R + m_b(R+h) + m_w R)g\sin z_1 + 2m_L g z_2\cos z_1 - 4m_L z_2 z_4 z_3 \\ 
2m_L g\sin z_1 - F_r + 2m_L z_2 z_3^2 
\end{bmatrix}
$$

$$
\dot{z} = \begin{bmatrix} 
z_3 \\ 
z_4 \\ 
M^{-1}(z_2) \cdot \left( H(z) + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u \right)
\end{bmatrix}
$$

**Coupleness analysis of the system**:

$$
A(z) = \frac{\partial f}{\partial z}(z), \quad B(z) = \frac{\partial f}{\partial u}(z)
$$

We can use symbolic toolbox to do that.

## Step 1: Find the strongest coupled state



