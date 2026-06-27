# Nonlinearity analysis

Given any nonlinear system, we can linearize it around a point and analyze the stability of the linearized system. However, the controller designed for the linearized system may not stabilize the original nonlinear system in all regions. Therefore, it is important to analyze the nonlinearity of the system and determine the regions where the linearized controller can stabilize the nonlinear system.

## Setup

Given any nonlinear system, it is of the form:

$$
\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^m
$$

To simplify the analysis, we assume the force input is linear in the input $u$, so the system can be expressed as:

$$
\dot{z} = f(z) + g(z) u = f(z) + Bu
$$

With the linearization of the system around a point $z_0$, we have:

$$
\dot{z} = f(z_0) + \frac{\partial f}{\partial z} (z_0) (z - z_0) + B u = A (z - z_0) + B u
$$

For further simplification, we can assume $z_0 = \hat{0}$, so the linearized system becomes:

$$
\dot{z} = A z + B u
$$

## Linearized controller

Given some cost function $J(z, u)$, we can design a linearized controller for the linearized system. The controller is of the form:

$$
u = -K z
$$

And:

$$
\dot{z} = (A - BK) z
$$

Depend on the cost function definition, the $K$ will have different values, however, ususal assumption of cost function of the form: $J(z, u) = \int_0^\infty (z^T Q z + u^T R u) dt$ may lead to limitations on time resonse thus not be able to cover all the nonlinear system behavior. 

Therefore, here we assume K is any random combination of the form $K = [k_1, k_2, \ldots, k_n], \qquad k_i \in \mathbb{R}$.

## Nonlinearity analysis

### Discretization

Use MPC ideaology, we can assume the system to be discretized with a time step $\Delta t$, and the system can be expressed as:

$$
\begin{aligned}
z_{k+1} &= z_k + \Delta t f(z_k) + \Delta t B u_k \\
\implies \frac{z_{k+1}}{\Delta t} &= \frac{z_k}{\Delta t} + f(z_k) + B u_k \\
\implies \frac{z_{k+1}}{\Delta t} &= f(z_k) + \left( \frac{1}{\Delta t} I_n - BK \right) z_k
\end{aligned}
$$

The linearized system can be expressed as:

$$
\frac{z_{k+1}}{\Delta t} = \frac{z_k}{\Delta t} + A z_k + B u_k =\frac{z_k}{\Delta t} + \frac{\partial f}{\partial z} z_k + B u_k \\
\implies \frac{z_{k+1}}{\Delta t} = \left( \frac{1}{\Delta t} I_n + A - BK \right) z_k = \left( \frac{1}{\Delta t} I_n + \frac{\partial f}{\partial z} - BK \right) z_k
$$

Normally we do:

$$
\left( \frac{z_{k+1}}{\Delta t} \right)_{\text{nonlinear}} - \left( \frac{z_{k+1}}{\Delta t} \right)_{\text{linear}} = f(z_k) - \frac{\partial f}{\partial z} z_k
$$

This cancels out the input, and decide that the nonlinearity of the system is determined by the difference between the nonlinear function $f(z_k)$ and its linear approximation $\frac{\partial f}{\partial z} z_k$, and is independent of the input $u_k$. However, this is not true in general, as the input $u_k$ can also affect the nonlinearity of the system. Therefore, we need to consider the effect of the input on the nonlinearity of the system.

## Nonlinearity measure

### Target

Go back to the original nonlinear system:

$$
\dot{z} = f(z) + Bu
$$

And the linearized system:

$$
\dot{z} = A z + Bu
$$

In mathematical terms, we want to find a state $z_c$ such that under the linearized control law $u_L(z) = -Kz$, the true nonlinear system escapes the controllable zone $Z_{controllable}$ in the next time step $\Delta t$, while the linearized model incorrectly predicts that the state remains safe. Crucially, to prove that this point is truly a part of the performance boundary (and not just an inherently uncontrollable state), there must exist some alternative feasible control input $u_{nl}$ that successfully keeps the nonlinear system within the zone.This gives us three conditions for $z_c$:

The linear controller fails on the true nonlinear system:

$$
z_c + \Delta t f(z_c) - \Delta t BK z_c \notin Z_{controllable}
$$

The linear prediction model claims the state is safe (Model Mismatch):
$$z_c + \Delta t A z_c - \Delta t BK z_c \in Z_{controllable}
$$

### Set theorm

Writen in set theory $\mathcal{R} = Z_{controllable}$

$$
\partial \mathcal{R} = \left\{ z_c \in \mathcal{R} \; \middle| \; 
\begin{aligned}
& z_c + \Delta t \big(f(z_c) - BK z_c\big) \notin \mathcal{R} \\
\text{and } & z_c + \Delta t \big(A - BK\big) z_c \in \mathcal{R}
\end{aligned}
\right\}
$$


This way it actually assumes that $\mathcal{R}$ is a convex set, and the boundary is a hyperplane. We can discuss the limitation of this assumption later, but for now we make this assumption to simplify the analysis, and this applyies to our unicycle system as well. 

### Quantify the nonlinearity

First some simplifications:

$$
\partial \mathcal{R} = \left\{ z_c \in \mathcal{R} \; \middle| \; 
\begin{aligned}
& H_1(z_c) \notin \mathcal{R} \\
\text{and } & H_2(z_c) \in \mathcal{R}
\end{aligned}
\right\}
$$

And our target is to find the expression of $G(z)$ with $H$ such that:

$$
\partial \mathcal{R} = \left\{ z \in \mathbb{R}^n \; \middle| \; G(z) = 0, G \in \mathbb{R}^n \to \mathbb{R} \right \}
$$

As $\mathcal{R}$ is a convex set, at the boundary, the direction must be either tangent or point inner to the set, so we can write:

$$
\partial \mathcal{R} = \left\{ z \in \mathbb{R}^n \; \middle| \; \langle \nabla G(z), H_1(z) - z \rangle \le 0 \quad \text{and} \quad \langle \nabla G(z), H_2(z) - z \rangle \ge 0 \right\}
$$

Usually they apply the Cauchy-Schwarz inequality to get a more general form:

$$
\| \nabla G(z) \| \cdot \| \phi(z) \| \ge \langle \nabla G(z), \phi(z) \rangle
$$

However this is not the tightest bound.

Give:

$$
f(z) = Az + \phi(z)
$$

And 

$$
\begin{aligned}
H_1(z) - z &= \Delta t \big(Az - BKz + \phi(z)\big) = \\
H_2(z) - z &= \Delta t \big(A - BK\big)z
\end{aligned}
$$

Good news is that we can eliminate the $\Delta t$ term, and we can write:

$$
\partial \mathcal{R} = \left\{ z \in \mathbb{R}^n \; \middle| \; \langle \nabla G(z), Az - BKz + \phi(z) \rangle \le 0 \quad \text{and} \quad \langle \nabla G(z), (A - BK)z \rangle \ge 0 \right\}
$$

This means that for certain $\lambda(z) \in [0, 1]$, we can write:

$$
\partial \mathcal{R} = \left\{ z \in \mathbb{R}^n \; \middle| \; \langle \nabla G(z), Az - BKz + \lambda(z)\phi(z) \rangle = 0 \right\}
$$

By expansion:

$$
\lambda(z) = - \frac{\langle \nabla G(z), (A - BK)z \rangle}{\langle \nabla G(z), \phi(z) \rangle}
$$

Then the equivalent form of the boundary is:

$$
\partial \mathcal{R} = \left\{ z \in \mathbb{R}^n \; \middle| \; \lambda(z) = - \frac{\langle \nabla G(z), (A - BK)z \rangle}{\langle \nabla G(z), \phi(z) \rangle}, \quad \lambda(z) \in [0, 1] \right\}
$$

### Find the boundary










