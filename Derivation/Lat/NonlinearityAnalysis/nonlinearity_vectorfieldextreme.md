# Boundary Characterization via Critical Invariant Manifolds

Given a continuous-time nonlinear closed-loop system:
$$
\dot{z} = A_{cl} z + \phi(z), \quad z \in \mathbb{R}^n
$$

To analyze the trajectory propagation geometrically, we define the forward one-step flow map $\psi(z) \in \mathbb{R}^n \to \mathbb{R}^n$ for an infinitesimal time increment $dt$:

$$
\psi(z) = z + \big( A_{cl} z + \phi(z) \big) dt
$$

We want to find the limit of the set where:

$$
\mathcal{C} = \left\{ z \in \mathbb{R}^n \; \middle| \; \psi^\tau(z) \in \mathcal{C}, \ \tau \in \mathbb{N} \right\}
$$

Here $\psi^\tau(z)$ is the $\tau$-th iteration of the flow map $\psi(z)$.

By calculating:

$$
\mathcal{C} = \left\{ z \in \mathbb{R}^n \; \middle| \; \psi^\tau(z) = 0, \ \tau \in \mathbb{N} \right\}
$$

we can find several points on the boundary of the controllable set, and then we can use these points to fit a polynomial to approximate the boundary of the controllable set.