# Nonlinearity analysis

Given any nonlinear system, we can linearize it around a point and analyze the stability of the linearized system. However, the controller designed for the linearized system may not stabilize the original nonlinear system in all regions. Therefore, it is important to analyze the nonlinearity of the system and determine the regions where the linearized controller can stabilize the nonlinear system.

[TOC]

## Setup

Given any nonlinear system, it is of the form:

$$\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^m$$

To simplify the analysis, we assume the force input is linear in the input $u$, so the system can be expressed as:

$$\dot{z} = f(z) + g(z) u = f(z) + Bu$$

With the linearization of the system around a point $z_0$, we have:

$$\dot{z} = f(z_0) + \frac{\partial f}{\partial z} (z_0) (z - z_0) + B u = A (z - z_0) + B u$$

For further simplification, we can assume $z_0 = \hat{0}$, so the linearized system becomes:

$$\dot{z} = A z + B u$$

## Linearized controller

Given some cost function $J(z, u)$, we can design a linearized controller for the linearized system. The controller is of the form:

$$u = -K z$$

And:

$$\dot{z} = (A - BK) z$$

Depend on the cost function definition, the $K$ will have different values. However, the usual assumption of a cost function of the form $J(z, u) = \int_0^\infty (z^T Q z + u^T R u) dt$ may lead to limitations on time response and thus not be able to cover all the nonlinear system behavior.

Therefore, here we assume $K$ is any random combination of the form $K = [k_1, k_2, \ldots, k_n], \quad k_i \in \mathbb{R}$.

## Nonlinearity measure

### Target

Go back to the original nonlinear system:

$$\dot{z} = f(z) + Bu$$

And the linearized system:

$$\dot{z} = A z + Bu$$

In mathematical terms, we want to find a state $z_c$ such that under the linearized control law $u_L(z) = -Kz$, the true nonlinear system escapes the controllable zone $Z_{controllable}$ in the next time step $\Delta t$, while the linearized model incorrectly predicts that the state remains safe. Crucially, to prove that this point is truly a part of the performance boundary (and not just an inherently uncontrollable state), there must exist some alternative feasible control input $u_{nl}$ that successfully keeps the nonlinear system within the zone.

This gives us the mismatch condition for $z_c$:

The linear controller fails on the true nonlinear system (Actual escape):

$$z_c + \Delta t f(z_c) - \Delta t BK z_c \notin Z_{controllable}$$

The linear prediction model claims the state is safe (Predicted containment):

$$z_c + \Delta t A z_c - \Delta t BK z_c \in Z_{controllable}$$

To be more precise, we are looking for the gap where:

* **Linear controller predicts safe** ($\mathcal{B}_1$)
* **Linear controller actually works** ($\mathcal{B}_2$)

**Our goal is to find $\mathcal{B}_1$ and $\mathcal{B}_2$.**

### Set theory

Written in set theory, $\mathcal{R} = Z_{controllable}$. $\mathcal{C}$ is the set of states where the linearized model incorrectly predicts that the system is controllable (safe), whereas the true nonlinear system is actually uncontrollable (escapes the safe zone) under the linear controller.

$$\mathcal{C} = \left\{ z_c \in \mathcal{R} \; \middle| \; z_c + \Delta t \big(f(z_c) - BK z_c\big) \notin \mathcal{R} \quad \text{and} \quad z_c + \Delta t \big(A - BK\big) z_c \in \mathcal{R} \right\}$$

With boundary definitions, $\mathcal{C}$ is in the middle of $\mathcal{B}_1$ and $\mathcal{B}_2$, where $\mathcal{B}_1$ is the stability boundary of the linearized model, and $\mathcal{B}_2$ is the stability boundary of the true nonlinear system under the same controller. This inherently assumes that the actual controllable set is bounded by $\mathcal{B}_2$ and the predicted set is bounded by $\mathcal{B}_1$.

## Quantify the nonlinearity

To express this in continuous time, we introduce a Lyapunov function candidate $V(z)$ to represent the safety/energy level set. The tendency of the system is given by the Lie derivative (directional derivative) of $V(z)$ along the system dynamics.

Let $A_{cl} = A - BK$ and let the nonlinear residual be $\phi(z) = f(z) - Az$. We define the evolution functions $G_1$ (linear prediction) and $G_2$ (true nonlinear dynamics) as:

$$
G_1(z) = \dot{V}_{lin}(z) = \langle \nabla V(z), A_{cl}z \rangle
$$

$$
G_2(z) = \dot{V}_{nl}(z) = \langle \nabla V(z), A_{cl}z + \phi(z) \rangle
$$

The critical set $\mathcal{C}$ where the linear controller is "deceived" by the nonlinearity is:

$$
\mathcal{C} = \left\{ z \in \mathbb{R}^n \; \middle| \; G_1(z) \le 0 \quad \text{and} \quad G_2(z) > 0 \right\}
$$

By this definition, the actual linearized-controllable set is:

$$
\mathcal{C}_l = \left\{ z \in \mathbb{R}^n \; \middle| \; G_1(z) \le 0 \right\}
$$

The boundaries are defined where the derivative is exactly zero: 

$$
\mathcal{B}_1 = \{ z \in \mathbb{R}^n \; | \; G_1(z) = 0 \} \quad \text{and} \quad \mathcal{B}_2 = \{ z \in \mathbb{R}^n \; | \; G_2(z) = 0 \}
$$

The difference between the two derivatives quantifies the nonlinearity of the system:

$$
G_2(z) - G_1(z) = \langle \nabla V(z), \phi(z) \rangle
$$

If $\langle \nabla V(z), \phi(z) \rangle \le 0$ for all $z \in \mathcal{R}$, then $\mathcal{C} = \emptyset$. This means the linearized controller can stabilize the nonlinear system in the whole controllable set, and the nonlinearity of the system is actually helping with the stabilization. We will focus on the case where $\mathcal{C} \neq \emptyset$.

### Linearized controllable set with boundary $\mathcal{B}_1$

Given a state-dependent Lyapunov matrix $V(z) = z^T P(z) z$, the boundary condition is defined by the tendency of the system being tangent to the level set surface:

$$G_1(z) = \langle \nabla V(z), \dot{z}_{lin} \rangle = 0$$

By applying the product rule and chain rule to $V(z)$ with $\dot{z}_{lin} = A_{cl}z$, we have:

$$G_1(z) = \dot{z}_{lin}^T P(z) z + z^T P(z) \dot{z}_{lin} + z^T \dot{P}_{lin}(z) z = 0$$

Where $\dot{P}_{lin}(z) = \sum_{i=1}^n \frac{\partial P(z)}{\partial z_i} (A_{cl} z)_i$ captures how the shape of the Lyapunov level set deforms along the linear trajectory.

$$\mathcal{B}_1 = \left\{ z \in \mathbb{R}^n \; \middle| \; z^T \big(P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \big) z = 0 \right\}$$

This means that linearized controllers will always assume they can control the entire $\mathbb{R}^n$ space if no external constraints exist. For purely linear time-invariant systems with a constant $P$, the region is indeed $\mathbb{R}^n$.

### Nonlinear controllable set with boundary $\mathcal{B}_2$

For the true nonlinear system, the dynamics are $\dot{z}_{nl} = A_{cl}z + \phi(z)$. The rate of change of the $P$ matrix along this true trajectory is $\dot{P}_{nl}(z) = \sum_{i=1}^n \frac{\partial P(z)}{\partial z_i} \dot{z}_{nl, i}$.

Given the boundary condition $G_2(z) = 0$, we expand this using the state-dependent $P(z)$:

$$
\mathcal{B}_2 = \left\{ z \in \mathbb{R}^n \; \middle| \; \dot{z}_{nl}^T P(z) z + z^T P(z) \dot{z}_{nl} + z^T \dot{P}_{nl}(z) z = 0 \right\}
$$

$$
\mathcal{B}_2 = \left\{ z \in \mathbb{R}^n \; \middle| \; z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z + 2 z^T P(z) \phi(z) = 0 \right\}
$$

Intrinsically, $z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z \leq 0$ represents the stability of the linear baseline, and $2 z^T P(z) \phi(z)$ is the nonlinearity of the system that contributes to the divergence. To find the maximal boundary exactly, this equation must be satisfied by a valid $P(z)$ topology, making the definition an optimization over candidate matrices:

$$
\mathcal{B}_2 = \left\{ z \in \mathbb{R}^n \; \middle| \; \max_{P(z)} \left[ z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z + 2 z^T P(z) \phi(z) \right] = 0 \right\}
$$

Solving the boundary $\mathcal{B}_2$: Notice that $z^T M z = \text{Tr}(M z z^T)$. We can separate the static matrix variables and the derivative constraints. Let $M(z) = A_{cl} z z^T + z z^T A_{cl}^T + \phi(z) z^T + z \phi(z)^T$.

$$z^T \big( P(z) A_{cl} + A_{cl}^T P(z) \big) z + 2 z^T P(z) \phi(z) + z^T \dot{P}_{nl}(z) z = \text{Tr}\big( P(z) \cdot M(z) \big) + \text{Tr}\big( \dot{P}_{nl}(z) z z^T \big)$$

So the boundary reduces to solving for a matrix function $P(z)$ such that:

$$\max_{P(z)} \left[ \text{Tr}\big( P(z) \cdot M(z) \big) + \text{Tr}\big( \dot{P}_{nl}(z) z z^T \big) \right] = 0$$

Here, $\text{Tr}\big( \dot{P}_{nl}(z) z z^T \big)$ serves as the mathematical "deformation penalty". It prevents the optimization from arbitrarily changing $P(z)$ to enclose unstable vectors by forcing the boundary's shape to evolve consistently with the system dynamics. The condition for the matrix function $P(z)$ over the valid domain is:

$$P(z) \succ 0$$

$$P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \prec 0 \quad \text{(stable linear baseline)}$$

$$- \big( P(z) \cdot M(z) + \dot{P}_{nl}(z) z z^T \big) \succeq 0 \quad (\text{from } \dot{V}(z) \le 0)$$