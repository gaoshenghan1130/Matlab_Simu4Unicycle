# Nonlinearity Analysis in Field Form

Given some zone where the controller is able to control the nonlinear system $\mathcal{R}$, we can define a function $G(z)$ such that:

$$
\mathcal{R} = \{ z \in \mathbb{R}^n \mid G(z) \ge 0 \}
$$

$$
\partial\mathcal{R} = \{ z \in \mathbb{R}^n \mid G(z) = 0 \}
$$

Vector field of the nonlinear system is:

$$
\boldsymbol{v}_{nl}(z) = A_{cl}z + \phi(z)
$$

So:

$$
\mathcal{R} =\left\{ z \in \mathbb{R}^n \mid \langle \nabla G(z), \boldsymbol{v}_{nl}(z) \rangle \ge -\alpha(G(z)) \right \}
$$

$$
\partial\mathcal{R} =\left\{ z \in \mathbb{R}^n \mid \langle \nabla G(z), \boldsymbol{v}_{nl}(z) \rangle = 0 \right \}
$$



Where $\alpha(0) = 0$ and $\alpha(\cdot)$ is an increasing function.

The requirement of $G(z)$ is loose, for any $G(z)$ the part $G(z)$ overlaps with the part of the constraint surface should be identical.

## Solving $\langle \nabla G(z), \boldsymbol{v}_{nl}(z) \rangle = 0$

Plug in the definition of $\boldsymbol{v}_{nl}(z)$:

$$
\langle \nabla G(z), A_{cl}z + \phi(z) \rangle = \langle \nabla G(z), A_{cl}z \rangle + \langle \nabla G(z), \phi(z) \rangle = 0
$$

Construction will always result in a optimization problem, that won't be good.

