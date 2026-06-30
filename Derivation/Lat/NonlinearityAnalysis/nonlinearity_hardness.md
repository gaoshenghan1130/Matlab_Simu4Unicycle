# 1. Proof that linearized controllable set boundary is an NP-hard problem

Now we know the controllable set boundary:

$$
\max_{P(z)} \left[ \text{Tr}\big( P(z) \cdot M(z) \big) + \text{Tr}\big( \dot{P}_{nl}(z) z z^T \big) \right] = 0
$$

And the region:

$$
P(z) \succ 0, \\
\quad P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \prec 0, \quad \text{(stable linear baseline)} \\
 \quad - \big( P(z) \cdot M(z) + \dot{P}_{nl}(z) z z^T \big) \succeq 0, \quad (\text{from } \dot{V}(x) \le 0)
$$

We want to prove that the problem of finding the boundary of the controllable set is NP-hard.

## Known Theorem: BMI

Verifying the feasibility of a Bilinear Matrix Inequality (BMI) is known to be NP-hard. A BMI is defined as:

$$
P \succ 0 \\
(A + BKC)^T P + P(A + BKC) \prec 0
$$

Verifying $P$ and $K$ that satisfy the above BMI is NP-hard.

In our case, we try to verify $P$ and $K$ that satisfy the following ($K$ is inside $M(z)$):

$$
P(z) \succ 0, \\
\quad P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \prec 0, \\
\quad - \big( P(z) \cdot M(z) + \dot{P}_{nl}(z) z z^T \big) \succeq 0, \\
\text{At the neighbour of } V(z) = z^T P(z) z
$$

## Proof

Let $\phi(z) = 0$, the equation will reduce to the same form as the BMI. Therefore, the problem of finding the boundary of the controllable set is at least NP-hard.

# 2. Proof that the linearized controllable set boundary is undecidable

We know that the problem of verifying the feasibility of a BMI is NP-hard but it is still decidable due to the Tarski–Seidenberg Quantifier Elimination Theorem: any first-order formula over the reals is equivalent to a quantifier-free formula. Therefore, we can always verify a solution to BMI in finite time. However, the problem of finding the boundary of the controllable set could be undecidable because we consider a nonlinear term: $\phi(z)$.

## Known Theorem: Halting Problem

The Halting Problem is a foundational undecidable problem in computer science, establishing that no general algorithm can determine whether an arbitrary computer program (a Turing machine) will eventually halt or run forever.

In their seminal 2000 paper, Blondel and Tsitsiklis demonstrated that for a general nonlinear system $\dot{z} = f(z, u)$, the problem of determining whether an initial state can be driven to the origin (i.e., determining the controllable set) is undecidable. Their proof relies on a reduction from the Halting Problem, showing that continuous non-polynomial dynamics can perfectly simulate a Universal Turing Machine (UTM).

Here, we extend this logic to rigorously prove a stronger claim: Even when the control input is strictly constrained to be linear, i.e., $u(z) = Kz$, the problem of finding the exact boundary of the controllable set remains undecidable.









