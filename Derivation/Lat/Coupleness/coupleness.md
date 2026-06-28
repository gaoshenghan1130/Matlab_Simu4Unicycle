# Coupleness analysis of nonlinear system

$$
\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^n
$$

To give a measure of the coupleness of the nonlinear system, the definition of coupleness must statisfy:

1. The coupleness is related only to the system itself and the current state. $\implies \mathcal{C} = \mathcal{C}(f, z) \in \mathbb(\mathbb{R}^n \times \mathbb{R}^n\to \mathbb{R}^n) \times \mathbb{R}^n \to \mathbb{R}^n \times \mathbb{R}^n$ 

2. $\mathcal{C}_{ij} = 0$ if and only if the $i$-th and $j$-th states are not coupled. i.e:

$$
\mathcal{C}_{ij} = 0 \iff \frac{\partial f_i}{\partial z_j} = 0 \quad\text{and} \quad \frac{\partial f_j}{\partial z_i} = 0
$$

3. The coupleness should also reflect the cross-channel control authority. That is, if $\mathcal{C}_{ij} = 0$, the $j$-th control input must not directly affect the $i$-th state dynamics:

$$
\mathcal{C}_{ij} = 0 \implies \frac{\partial f_i}{\partial u_j} = 0 \quad \text{and} \quad \frac{\partial f_j}{\partial u_i} = 0
$$

4. Transitivity via Cascade Dynamics: If channel $i$ is coupled to $j$ ($C_{ij}$), and $j$ is coupled to $k$ ($C_{jk}$), the indirect coupleness from $i$ to $k$ is governed by the composition of their respective coupleness measures, yielding:

$$
\mathcal{C}_{ik}(z) = \sum_{j=1}^n \mathcal{C}_{ij}(z) \mathcal{C}_{jk}(z)
$$

5. Covariance and Absorption under Diffeomorphism: Under any smooth coordinate transformation $z^* = T(z)$, the coupleness matrix maps covariantly via the system's Jacobian matrix. Specifically, if $T(z)$ is constructed along the integral manifold of the system's invariant distribution to achieve geometric decoupling (i.e., $\frac{\partial f^*_i}{\partial z^*_j} = 0$), the intrinsic coupling between states $i$ and $j$ in the original system is entirely absorbed by the topology of the new coordinate framework, strictly yielding $\mathcal{C}^*_{ij} = 0$.

6. If the system is overlapped with two different physical systems, the coupleness should be able to reflect the coupleness of the two systems. i.e. $f(z, u) = f^{(1)}(z, u) + f^{(2)}(z, u)$ means:

$$
\|\mathcal{C}(f^{(1)} + f^{(2)}, z)\| \le \|\mathcal{C}(f^{(1)}, z)\| + \|\mathcal{C}(f^{(2)}, z)\|
$$

Define:

$$
A(z) =  \frac{\partial f}{\partial z}(z) , \quad B(z) =  \frac{\partial f}{\partial u}(z) 
$$

To satisfy requirement 4:

$$
\mathcal{C}(f, z) = I + A(z) + A^2(z) + A^3(z) + \cdots = (I - A(z))^{-1}
$$

We need an additional parameter $\gamma$ to cover all possible cases, so we have:

$$
\mathcal{C}^z(z) = \gamma A(z) + \gamma^2 A^2(z) + \gamma^3 A^3(z) + \cdots \\
\implies \mathcal{C}^z(z) = (I - \gamma A(z))^{-1} - I
$$

$$
\mathcal{C}^u(z) = \mathcal{C}^z(z) \cdot B(z)
$$
