# Dynamic Coupleness Analysis for Nonlinear Control Systems

Consider a general nonlinear system:
$$\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^m$$

To evaluate the net dynamic interaction and control authority for feedback design, we map the system into a **state-dependent directed weighted graph**, denoted as $\mathcal{G}(z, u) = (\mathcal{V}, \mathcal{E}, \mathcal{W})$. 
* **Nodes $\mathcal{V}$**: State nodes $z_i$ and input nodes $u_j$.
* **Edges and Weights $\mathcal{W}$**: The directed edge weight reflects the dynamic gain (excitation or inhibition) from one node to another. 

Unlike purely structural topology, we define the "Dynamic Coupleness" matrix $\mathcal{C}$ to capture the **net effective interaction** across the network, preserving the signs of physical influences.

---

## 1. Axioms for Dynamic Coupleness

**Axiom 1: Topological & State Dependency**
The coupleness is an intrinsic property determined by the system's dynamic gradients evaluated at the current state and input:
$$\mathcal{C} = \mathcal{C}(\mathcal{G}(z, u))$$

**Axiom 2: Direct Dynamic Coupling (Local Gradients)**
The direct effective coupling from state $j$ to state $i$ is defined by the localized system Jacobian, preserving the directional gain (positive for excitation, negative for inhibition):
$$\mathcal{C}_{ij}^{\text{direct}} = A_{ij}(z, u) = \frac{\partial f_i}{\partial z_j}$$

**Axiom 3: Direct Input Authority**
Control inputs act as external source nodes. The direct authority of input $j$ over state $i$ is defined by the input gradient:
$$\mathcal{C}_{ij}^{\text{input}} = B_{ij}(z, u) = \frac{\partial f_i}{\partial u_j}$$

**Axiom 4: Transitivity via Net Dynamic Walks**
In a interconnected system, $j$ influences $i$ indirectly through intermediate states. The global coupleness $\mathcal{C}_{ij}$ is the **weighted sum of all directed paths**, allowing for dynamic cancellations (e.g., if parallel paths exert equal but opposite effects, the net indirect coupling is zero).

---

## 2. Analytic Derivation

To mathematically realize Axiom 4 (Transitivity) while capturing both state and input networks, we define the **Augmented Dynamic Matrix**:
$$\tilde{A}(z, u) = \begin{bmatrix} A(z, u) & B(z, u) \\ 0 & 0 \end{bmatrix}$$

The matrix $\tilde{A}^k$ explicitly computes the net dynamic gain accumulated over all paths of exactly length $k$. The global dynamic coupleness matrix $\tilde{\mathcal{C}}$, accounting for all feedback loops of infinite lengths, is constructed using a Katz-like resolvent series:
$$\tilde{\mathcal{C}}(z, u) = \sum_{k=1}^\infty \gamma^k \tilde{A}^k$$
Where $\gamma > 0$ is an attenuation factor scaling higher-order dynamic interactions, ensuring convergence ($\gamma < 1/\|\tilde{A}\|$). 

This yields the closed-form global coupleness:
$$\tilde{\mathcal{C}}(z, u) = (I - \gamma \tilde{A})^{-1} - I$$

---

## 3. Extraction for Controller Design

By partitioning the global coupleness matrix $\tilde{\mathcal{C}}$, we extract the two critical matrices for control design:
$$\tilde{\mathcal{C}} = \begin{bmatrix} \mathcal{C}^z & \mathcal{C}^u \\ 0 & 0 \end{bmatrix}$$

**1. State-to-State Dynamic Coupleness ($\mathcal{C}^z$):**
Captures the internal cross-coupling and internal feedback loops among states:
$$\mathcal{C}^z(z, u) = (I - \gamma A)^{-1} - I$$

**2. Effective Control Authority ($\mathcal{C}^u$):**
Extracts the global input-to-state authority, analytically yielding:
$$\mathcal{C}^u(z, u) = \gamma (\mathcal{C}^z + I) B$$

**Control Implication:** This elegantly shows that the *Effective Control Authority* ($\mathcal{C}^u$) is not just the direct input matrix $B$, but $B$ dynamically modulated and distributed by the system's internal state coupling network $(\mathcal{C}^z + I)$. This provides a theoretical foundation for designing decoupling controllers based on the exact inverse of $\mathcal{C}^u$.