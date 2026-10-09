# Critical-speed study with progressively added control

## Purpose

Starting from the current upright straight-rolling model, add feedback terms
one at a time and determine whether each one lowers the low-speed stability
boundary. Recompute the closed-loop eigenvalues and both speed boundaries
after every control change; nonlinear trajectories validate, but do not
replace, the eigenvalue calculation.

## Fixed model and comparison rule

Use the same mate_current_mass model parameters for every case:
\(R=0.253\,\mathrm m\), \(m_w=2.436\,\mathrm{kg}\),
\(m_r=2.3\,\mathrm{kg}\), \(m_p=2.799\,\mathrm{kg}\),
\(h=0.025\,\mathrm m\), \(B_R=B_P=0\), and the inertias from
model_parameters.m. Linearize at upright straight rolling for each
forward speed \(v\), with \(\sigma_2=v/R\).

For each controller case \(j\), form the state-feedback closed-loop
linearization

\[
A_{\mathrm{cl},j}(v)=A(v)+B(v)K_j,
\qquad
\alpha_j(v)=\max_{\lambda\in\Lambda_{\mathrm{dyn},j}(v)}
                    \Re\lambda .
\]

Here \(K_j\) is the derivative of that case's implemented input law
\([F,M_2]\) with respect to the full model state. Find and report the
**upper** speed boundary where \(\alpha_j(v)\) changes from positive to
nonpositive. Report the eigenvalues closest to the imaginary axis at the
boundary, including their imaginary parts/frequency. Exclude only known
zero eigenvalues associated with phase/position symmetries; do not discard
nonzero poles just because their real part is small. Check the full spectrum
on both sides of every reported boundary.

Keep the model, controller gains, speed range, eigenvalue tolerance,
initial perturbation, integration duration, and plot limits fixed across
comparisons. If there are multiple unstable/stable intervals, report each
crossing and identify the upper boundary explicitly. Use the same nonlinear
initial state and run below, near, and above each candidate boundary as a
separate validation.

## Control sequence

| Case | Force \(F\) | Torque \(M_2\) | Status |
|---|---|---|---|
| 0 — reference | \(0\) | \(\;3\gamma+0.8\dot\gamma\) | Existing eigenvalue baseline: \(v_{\mathrm{crit,high}}\approx1.723\,\mathrm{m/s}\). |
| 1 — center rod | \(854.34418(0-r)+105.568949(0-\dot r)\) | Same gamma PD | Current nonlinear run has this \(r\)-PD, but its eigenvalue boundary has **not** yet been recomputed. This is the next comparison. |
| 2 — add lean feedback | Case 1 force plus \(K_\theta(0-\theta)+D_\theta(0-\dot\theta)\) | Same gamma PD | To test after Case 1. Define gains and sign convention explicitly before calculating eigenvalues. |
| 3 — tune combined lateral PD | Retune \(r\) and lean terms together, changing one gain at a time | Same gamma PD | Keep each gain set as a separate case; do not report a retuned case as a single-term addition. |
| 4 — alternative gamma regulation | Best lateral law from above | Replace gamma PD with exact \(\gamma=0\) constraint, if tested | This is a controller replacement, not an additional independent actuator. Recompute the spectrum from its own linearized input law. |

Coordinates and rates are those of the model:

\[
\dot r=R\sigma_1+\sigma_r,\qquad
\dot\theta=\sigma_1,\qquad
\dot\gamma=\sigma_\gamma-\sigma_3\tan\theta.
\]

At the upright operating point, \(\dot\gamma\) linearizes to
\(\sigma_\gamma\). The \(r\)-centering force is therefore

\[
F=-K_r r-D_r(R\sigma_1+\sigma_r),
\qquad K_r=854.34418,\quad D_r=105.568949.
\]

The previously quoted \(1.723\,\mathrm{m/s}\) result used \(m_r=1\,\mathrm{kg}\).
All results below use \(m_r=2.3\,\mathrm{kg}\), and the reference case is
recomputed with the same full-state eigenvalue procedure as every controlled
case.

## Results log

| Case | Active feedback | Gains | Linear low boundary (m/s) | Other linear boundary (m/s) | Nonlinear validation at low boundary | Finding |
|---|---|---|---:|---:|---|---|
| 0 | Gamma PD only | \(K_{p\gamma}=3,\ K_{d\gamma}=0.8\) | No crossing in 0.05–20 | No crossing in 0.05–20 | No candidate low boundary | No stable interval detected. The earlier 1.723 m/s result used \(m_r=1\), so it is not the reference for this study. |
| 1 | Gamma PD + \(r\)-PD | \(K_r=854.34418,\ D_r=105.568949\) | **1.588584** | 3.144993 | All three runs remain bounded and oscillatory for 30 s; peak \(|\theta|=17.30^\circ/10.48^\circ/4.59^\circ\) | Lowest linearized boundary among tested cases; the simulation does not show runaway. |
| 2 | Case 1 + lean PD | \(K_\theta=20,\ D_\theta=4\) | 5.752547 | No crossing up to 20 | Peak \(|\theta|=0.118^\circ/0.117^\circ/0.116^\circ\) | Raises the low-speed boundary substantially; helps the high-speed stable region but does not meet the low-boundary objective. |
| 3a | Case 1, lower rod stiffness | \(K_r=0.75(854.34418),\ D_r=105.568949\) | 1.589759 | 3.144993 | Peak \(|\theta|=17.25^\circ/10.42^\circ/4.55^\circ\) | Slightly worse than Case 1. |
| 3b | Case 1, higher rod damping | \(K_r=854.34418,\ D_r=1.25(105.568949)\) | 1.588584 | 3.144993 | Peak \(|\theta|=17.28^\circ/10.46^\circ/4.58^\circ\) | Same low boundary within numerical resolution. |
| 3c | Case 2, higher lean stiffness | \(K_\theta=40,\ D_\theta=4\) | 1.630032 | No crossing up to 20 | Peak \(|\theta|=17.37^\circ/10.43^\circ/4.51^\circ\) | Worse than Case 1. |
| 3d | Case 2, higher lean damping | \(K_\theta=20,\ D_\theta=8\) | 6.581542 | No crossing up to 20 | Peak \(|\theta|=0.115^\circ/0.113^\circ/0.112^\circ\) | Worse for low-speed operation. |
| 4 | Best tested lateral law (Case 1), exact \(\gamma=0\) constraint replacing gamma PD | Constraint rate 10 s\(^{-1}\) | 1.588584 | 3.144993 | Peak \(|\theta|=17.41^\circ/10.53^\circ/4.59^\circ\); \(|\gamma|<10^{-11}\) deg | Enforcing gamma exactly does not lower the low boundary. |

The MATLAB result file stores every dynamic eigenvalue at each crossing and
at fixed offsets immediately below and above it. The same file stores all
nonlinear traces of \(\theta,r,\gamma\), and forward speed for the three
matched test speeds. Low-boundary poles in Cases 1, 3a, 3b, and 4 are real
and near zero, rather than an oscillatory pair; the boundary frequency is
therefore 0 Hz.

The Case-1 nonlinear traces do **not** diverge during the 30 s runs. They
show sustained finite-amplitude oscillations. The run initialized 2% below
the linear boundary starts at 1.5568 m/s but its speed varies up to
1.6204 m/s, crossing the 1.5886 m/s boundary during the run. The run started
at the boundary also moves above it, while the +2% run remains above it.
Thus these trajectories do not test a constant-speed equilibrium below the
boundary. The eigenvalue crossing is a local linear stability result; it
does not imply unbounded motion in the nonlinear model. A finite-amplitude
bounded oscillation is consistent with nonlinear saturation or a limit
cycle, and the 30 s traces alone do not establish asymptotic convergence.

## Conclusion for the low-speed boundary

Among the tested laws, Case 1 gives the lowest linearized low-speed boundary,
\(1.588584\,\mathrm{m/s}\). Increasing rod damping by 25% leaves it
unchanged to the reported precision. The tested lean-feedback changes and
exact gamma-zero constraint do not lower it. The next useful tuning sweep
should vary \(K_r\) and \(D_r\) jointly around Case 1 while retaining the
same low-speed crossing definition and nonlinear perturbation.

## Nonlinear validation settings

All boundary validation simulations use the same \(0.1^\circ\) initial
lean, centered rod, zero initial lateral and gamma rates, 30 s duration, and
speeds at -2%, 0%, and +2% of each detected low boundary. The complete
traces and side-of-boundary spectra are stored in
`MATLAB/Full/analysis/critical_speed_control_results.mat`.

## Nonlinear tilt-event scan

To distinguish bounded oscillation from actual loss of balance, Case 1 was
also scanned over 44 initial speeds from 0.05 to 1.72 m/s. Each scan run
used the same \(0.1^\circ\) initial lean and was integrated for 90 s; reaching
the model's \(80^\circ\) tilt event counts as divergence. A speed of
1.42 m/s reached the event at 88.17 s, while 1.44 m/s stayed below the event
for 90 s. Since that result could depend on the observation duration, the
candidate boundary was tested for 1000 s:

- At 1.501900 m/s, the trajectory reached \(80^\circ\) at 647.050 s.
- At 1.501950 m/s, the trajectory stayed below \(80^\circ\) for 1000 s; its
  peak lean was 27.12°.

Thus the event transition for this initial condition and 1000 s horizon is
bracketed by 1.501900–1.501950 m/s. This is about 0.0867 m/s below the
1.588584 m/s linear eigenvalue boundary. It is a finite-time, nonlinear
tilt-event threshold, not a proof of permanent stability above the bracket.
The speed is not held fixed during these runs: in the 1.501900 m/s case it
reaches about 17.6 m/s before the tilt event. The result therefore describes
the implemented closed-loop model and initial condition, including its
speed drift.

The speed scan and figure are reproducible with
`MATLAB/Full/analysis/scan_case1_nonlinear_divergence.m`; outputs are saved
in `MATLAB/Full/analysis/case1_nonlinear_divergence_scan.mat` and `.png`.
