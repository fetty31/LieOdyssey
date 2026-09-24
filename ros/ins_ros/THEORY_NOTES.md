# iESEKF Covariance, SGal(3), Right-Plus Perturbations, and ROS Covariance
1. What does the iESEKF covariance actually represent?

In the iESEKF, the covariance matrix P is not directly a covariance of physical quantities expressed in world coordinates.

Instead, it is the covariance of the error state expressed in the tangent space of the current nominal state.

For an SGal(3) state

$$ X = (p,R,v,t) $$

with tangent coordinates

$$ \xi = \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix} $$

the interpretation is approximately:

- $\rho$: position error coordinates

- $\nu$: velocity error coordinates

- $\theta$: rotational error coordinates

- $s$: time error

The exact meaning of these coordinates depends on the Lie-group perturbation convention.

In this implementation, the perturbation is right-plus:

$$ X_{\text{true}} = X \oplus \delta x = X\operatorname{Exp}(\delta x) $$

Therefore,

$$ P = \operatorname{Cov}(\delta x) $$

is a covariance in the current right tangent coordinates.

2. Why does this matter for position and velocity?

For a conventional Euclidean state, it is easy to think of a covariance as

$$ P = \operatorname{Cov} \begin{bmatrix} \delta p\\ \delta v\\ \delta\theta \end{bmatrix}. $$

With a Lie-group state this is not necessarily true.

The tangent vector

$$ \xi = \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix} $$

is a local coordinate in the Lie algebra.

For SGal(3), the position and velocity components of the tangent are related to physical perturbations through the current rotation and, for position, also through the time component.

Under the right-plus convention, to first order:

$$ \delta p_I \approx R\rho + v s $$ $$ \delta v_I \approx R\nu $$ $$ \delta\theta_I \approx R\theta $$ $$ \delta t = s. $$

Here the subscript I denotes inertial/world-frame quantities.

Therefore, even if the state itself contains inertial-frame position and velocity, the tangent coordinates are not necessarily directly equivalent to those physical perturbations.

3. Tangent-space ordering

In manif's implementation the tangent ordering is

$$ \xi = \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix} $$

so the covariance is

$$ P = \operatorname{Cov} \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix}. $$

Thus the covariance blocks are conceptually

$$ P = \begin{bmatrix} P_{\rho\rho} & P_{\rho\nu} & P_{\rho\theta} & P_{\rho s}\\ P_{\nu\rho} & P_{\nu\nu} & P_{\nu\theta} & P_{\nu s}\\ P_{\theta\rho} & P_{\theta\nu} & P_{\theta\theta} & P_{\theta s}\\ P_{s\rho} & P_{s\nu} & P_{s\theta} & P_{ss} \end{bmatrix}. $$

This is the covariance of the Lie-algebra perturbation, not automatically the covariance of

$$ [p_I,v_I,\theta_I,t]. $$
4. Does the covariance need to be remapped after every update?

No.

This is an important distinction.

LieOdyssey's iESEKF internally does not need to repeatedly convert

$$ P_{\text{tangent}} \rightarrow P_{\text{world}} \rightarrow P_{\text{tangent}} $$

after every prediction or update.

The covariance remains in tangent coordinates throughout the filter.

For example:

          tangent covariance
                P_k
                 |
                 | prediction
                 v
          tangent covariance
                P_k+
                 |
                 | measurement update
                 v
          tangent covariance
                P_k
                 |
                 | prediction
                 v
          tangent covariance
                P_k+

What changes is which nominal state the tangent space is attached to.

5. Why does Adj(Exp(xi))^-1 appear during prediction?

This is the key point.

LieOdyssey's prediction contains:

```
X_.plus(f_(*this, imu) * Scalar(imu.dt), J_dX, J_xi);
```

and the implementation gives
```
J_dX = Adj(exp(xi))^-1
```
for the right-plus perturbation.

Suppose the nominal state is

$$ X_k $$

and the true state is represented as

$$ X_k^{true} = X_k\operatorname{Exp}(\delta x_k). $$

The nominal prediction is

$$ X_{k+1} = X_k\operatorname{Exp}(\xi_k). $$

Ignoring the state dependence of the dynamics for a moment, the true state propagates as

$$ X_{k+1}^{true} = X_k \operatorname{Exp}(\delta x_k) \operatorname{Exp}(\xi_k). $$

But we want to express the new error relative to the new nominal state:

$$ X_{k+1}^{true} = X_{k+1} \operatorname{Exp}(\delta x_{k+1}). $$

Therefore,

$$ X_k \operatorname{Exp}(\delta x_k) \operatorname{Exp}(\xi_k) = X_k \operatorname{Exp}(\xi_k) \operatorname{Exp}(\delta x_{k+1}). $$

Canceling \(X_k\),

$$ \operatorname{Exp}(\delta x_k) \operatorname{Exp}(\xi_k) = \operatorname{Exp}(\xi_k) \operatorname{Exp}(\delta x_{k+1}). $$

Hence

$$ \operatorname{Exp}(\delta x_{k+1}) = \operatorname{Exp}(-\xi_k) \operatorname{Exp}(\delta x_k) \operatorname{Exp}(\xi_k). $$

Using the adjoint identity,

$$ \operatorname{Exp}(-\xi) \operatorname{Exp}(\delta x) \operatorname{Exp}(\xi) \approx \operatorname{Exp} \left( \operatorname{Ad}_{\operatorname{Exp}(\xi)}^{-1} \delta x \right), $$

so

$$ \boxed{ \delta x_{k+1} \approx \operatorname{Ad}_{\operatorname{Exp}(\xi_k)}^{-1} \delta x_k } $$

which explains
```
J_dX = Adj(exp(xi))^-1
```
6. The adjoint is NOT converting the covariance to world coordinates

This distinction is extremely important.

Adj(exp(xi))^-1 does not mean:

"convert the covariance from body coordinates to world coordinates."

Instead, it means:

"express the same physical perturbation relative to the new nominal state, using the new tangent coordinates."

So the adjoint is a tangent-space transport operation.

Conceptually:
```
old nominal state X_k
        |
        | error expressed in T_{X_k}G
        |
        v
      δx_k
        |
        | Ad(exp(xi))^-1
        v
      δx_k+1
        |
        | expressed relative to
        | new nominal X_k+
        v
new nominal state X_k+
```

The covariance therefore remains a tangent covariance:

$$ P_k = \operatorname{Cov}(\delta x_k) $$

becomes

$$ P_{k+1}^- = \operatorname{Cov}(\delta x_{k+1}). $$

It has not been converted into a physical/world covariance.

7. What does J_xi represent?

LieOdyssey's prediction is more complete than the simple adjoint transport because the increment itself depends on the state.

We have:
```
Jacobian J_dX;
Jacobian J_xi;

X_.plus(
    f_(*this, imu) * Scalar(imu.dt),
    J_dX,
    J_xi
);
```
and then
```
Jacobian Fx =
    J_dX +
    J_xi * f_dx_(*this, imu) * Scalar(imu.dt);

MappingMatrix Fw =
    J_xi *
    f_dw_(*this, imu) * Scalar(imu.dt);
```
Here:

- J_dX accounts for the fact that the old perturbation must be transported through the group increment.
- J_xi describes how the resulting state changes when the Lie-algebra increment changes.
- f_dx_ describes how the dynamics depend on the current state.
- f_dw_ describes how the dynamics depend on process noise.

Therefore,

$$ \delta x_{k+1} = F_x\delta x_k + F_w\delta w_k. $$

with

$$ F_x = J_{dX} + J_\xi f_x\Delta t $$

and

$$ F_w = J_\xi f_w\Delta t. $$

The covariance propagation is then

$$ \boxed{ P_{k+1}^- = F_xP_kF_x^T + F_wQF_w^T } $$

8. What does J_xi = Jr mean?

Because the state update is

$$ X^+ = X\operatorname{Exp}(\xi), $$

the derivative with respect to the increment \(\xi\) involves the differential of the exponential map.

For a Lie group this is related to the right Jacobian:

$$ J_r(\xi). $$

Therefore, conceptually:

$$ J_\xi = \frac{\partial(X\oplus\xi)}{\partial\xi} $$

is the mapping from a perturbation of the Lie-algebra increment to a perturbation in the tangent space at the resulting state.

9. Why can P remain tangent even though the state is propagated in the inertial frame?

Because the state frame and the error-coordinate frame are different concepts.

For example, the state may contain:

$$ p_I,\quad v_I,\quad R_{IB}. $$

That says where the nominal state lives and how its quantities are represented.

But the error state can still be parameterized as

$$ \delta x = [\rho,\nu,\theta,s] $$

where the tangent coordinates are related to the group structure.

The Lie group defines how these local perturbations act on the nominal state.

Thus:

- Nominal state:
    p, v, R
    expressed according to the state convention

- Error state:
    $\rho$, $\nu$, $\theta$, $s$
    expressed in the tangent coordinates of SGal(3)

- Covariance:
    Cov([$\rho$, $\nu$, $\theta$, $s$])

10. So when DO we need the coordinate transformation?

We need it when an external interface expects a covariance in a different coordinate system.

ROS is the important example.

The filter has

$$ P_{\text{filter}} = \operatorname{Cov} \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix}. $$

Suppose we want to publish a ROS pose covariance representing physical inertial-frame position and orientation uncertainty.

Then we need a Jacobian

$$ J_{\text{ROS}\leftarrow\xi} = \frac{\partial \begin{bmatrix} \delta p_I\\ \delta\theta_I \end{bmatrix}} {\partial \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix}}. $$

Then

$$ \boxed{ P_{\text{ROS}} = J_{\text{ROS}\leftarrow\xi} P_{\text{filter}} J_{\text{ROS}\leftarrow\xi}^T } $$

This is the appropriate place for the coordinate transformation.

11. Physical perturbation mapping for SGal(3)

With

$$ \xi = \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix} $$

and right-plus perturbation, the first-order physical perturbations are

$$ \delta p_I \approx R\rho+vs, $$ $$ \delta v_I \approx R\nu, $$ $$ \delta\theta_I \approx R\theta, $$ $$ \delta t=s. $$

Therefore,

$$ \begin{bmatrix} \delta p_I\\ \delta v_I\\ \delta\theta_I\\ \delta t \end{bmatrix} = T(X) \begin{bmatrix} \rho\\ \nu\\ \theta\\ s \end{bmatrix} $$

with

$$ \boxed{ T(X)= \begin{bmatrix} R&0&0&v\\ 0&R&0&0\\ 0&0&R&0\\ 0&0&0&1 \end{bmatrix} } $$

to first order.

Consequently,

$$ \boxed{ P_{\text{physical}} = T(X) P_{\text{tangent}} T(X)^T } $$
12. ROS pose covariance

For geometry_msgs/PoseWithCovariance, the state contains only position and orientation.

The corresponding Jacobian is therefore

$$ \boxed{ T_{\text{pose}}(X) = \begin{bmatrix} R&0&0&v\\ 0&0&R&0 \end{bmatrix} } $$

assuming the ROS ordering is

$$ [p_x,p_y,p_z,\theta_x,\theta_y,\theta_z] $$

and the internal ordering is

$$ [\rho,\nu,\theta,s]. $$

Then

$$ \boxed{ P_{\text{pose}} = T_{\text{pose}} P T_{\text{pose}}^T } $$

This is the covariance that should be used if the goal is to publish a covariance describing the physical inertial-frame pose perturbations.

13. Why the velocity can affect position covariance

One particularly interesting consequence is the v s term:

$$ \delta p_I = R\rho+vs. $$

This means that because SGal(3) includes time as part of the group structure, a perturbation in the time coordinate can induce a position perturbation proportional to velocity.

Therefore the position covariance can contain contributions from:

$$ P_{\rho\rho}, \quad P_{\rho s}, \quad P_{ss}. $$

Expanding:

$$ P_{pp} = R P_{\rho\rho}R^T + R P_{\rho s}v^T + vP_{s\rho}R^T + vP_{ss}v^T. $$

This is another reason why simply taking the top-left 3×3 block of the tangent covariance is not generally equivalent to physical position covariance.

14. Important distinction: adjoint vs TPTᵀ

There are two completely different operations here.

A. Adjoint transport inside the filter

During prediction:

$$ \delta x_{k+1} = \operatorname{Ad}_{\operatorname{Exp}(\xi)}^{-1} \delta x_k +\cdots $$

and therefore

$$ P_{k+1} = F_xP_kF_x^T+\cdots $$

This keeps the covariance in Lie-algebra/tangent coordinates.

The adjoint accounts for the fact that the tangent space is attached to a new nominal state.

B. Physical-coordinate conversion for publication

When publishing:

$$ P_{\text{physical}} = TP_{\text{tangent}}T^T. $$

This changes the coordinate representation of the uncertainty.

It is not part of the filter's internal covariance propagation.

15. Complete picture

The whole architecture can therefore be understood as:
```
                  ┌───────────────────────┐
                  │   Nominal SGal(3)    │
                  │      state X_k       │
                  └───────────┬───────────┘
                              │
                              │ right perturbation
                              │
                              v
                  ┌───────────────────────┐
                  │  δx_k = [ρ ν θ s]    │
                  │   tangent coordinates │
                  └───────────┬───────────┘
                              │
                              │ covariance
                              v
                  ┌───────────────────────┐
                  │         P_k           │
                  │ Cov(δx_k)             │
                  └───────────┬───────────┘
                              │
                              │ prediction
                              │
                    Fx = JdX + Jxi fx dt
                    Fw = Jxi fw dt
                              │
                              v
                  ┌───────────────────────┐
                  │        P_k+           │
                  │ Cov(δx_k+)            │
                  │                       │
                  │ still tangent-space   │
                  │ covariance             │
                  └───────────┬───────────┘
                              │
                              │ measurement update
                              v
                  ┌───────────────────────┐
                  │        P_k             │
                  │ current tangent       │
                  │ covariance             │
                  └───────────┬───────────┘
                              │
                              │ only for publication
                              │
                              │ P_phys = T P Tᵀ
                              v
                  ┌───────────────────────┐
                  │ Physical/world-frame  │
                  │ covariance             │
                  └───────────┬───────────┘
                              │
                              v
                        ROS message
```
16. What J_dX is really doing

A useful way to remember it is:

Adj(exp(xi))^-1 does not transform the covariance into another physical frame. It transforms the coordinates of the error because the nominal state has moved on the Lie group.

At time \(k\):

$$ \delta x_k \in T_eG $$

when represented through the right-trivialized error convention.

After propagation:

$$ \delta x_{k+1} \in T_eG $$

again, but it describes the error relative to a different nominal state.

The adjoint is what relates those two tangent representations.

17. Why this is especially relevant for right-plus

For right-plus,

$$ X_{\text{true}} = X\operatorname{Exp}(\delta x). $$

When the nominal state moves by

$$ X^+ = X\operatorname{Exp}(\xi), $$

the old perturbation is effectively moved from the left side of the new increment to the right side:

$$ \operatorname{Exp}(\delta x) \operatorname{Exp}(\xi) = \operatorname{Exp}(\xi) \operatorname{Exp}(\delta x^+). $$

This produces

$$ \delta x^+ = \operatorname{Ad}_{\operatorname{Exp}(\xi)}^{-1} \delta x. $$

For a left-plus convention, the corresponding transport relationship would be different.

So the appearance of

Adj(exp(xi)).inverse()

is a direct consequence of the right-plus error convention.