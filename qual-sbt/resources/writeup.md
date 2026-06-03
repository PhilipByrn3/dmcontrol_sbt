Shell *et al.*: Bare Demo of IEEEtran.cls for IEEE Journals

::: IEEEkeywords
Simulation, Split-Belt Treadmill
:::

# Introduction

-belt treadmills have become a widely used research tool to study human
gait adaptation, step length asymmetries, symmetric gaits, adaptation to
new motor control environments, and have potential uses for post-stroke
rehabilitation. However, the split-belt treadmill is commercially
expensive and can require a harness setup for use with human subjects,
leading to difficulty for applications in the real world. The split-belt
treadmill has become a useful tool for researchers, but the high cost of
entry for split-belt treadmill applications limits use.\
The development and application of reinforcement learning algorithms to
real-world problems has furthered our understanding of motor control,
and high-fidelity, high-throughput environments allow researchers to
experiment with real-world scenarios such as human walking without
involving human subjects and implementing novel techniques. One of the
experimental tools which is leading to novel discoveries about motor
control and adaptation is the split-belt treadmill, of which, no
experimentally-validated environments currently exist. To capitalize on
the benefits from utilizing high-throughput simulations, a high-fidelity
environment is needed to create a realistic expectation of results. The
purpose of this study was to create an experimentally validated,
high-throughput, and high-fidelity environment created in MuJoCo, which
serves as a step forward to alleviate the high barrier of entry to
split-belt treadmill research, will allow researchers to experiment with
new techniques, and obtain realistic outcomes of their experimentation.

![An ideal rimless wheel.](rimless_wheel_diagram_figure.pdf){#fig_sim
width="3.4in"}

# Methods

::: figure*
![image](sbt_figure_mjc_rimless.pdf){width="\\textwidth"}
:::

This environment is made using the dmcontrol library for MuJoCo and the
intended use is for realistic, high throughput simulations involving a
split-belt treadmill. In order to give this environment scientific
credence, a model of a rimless wheel is placed on the split-belt
treadmill with one belt remaining stationary while one belt increases in
velocity. This system of a rimless wheel and split-belt treadmill was
studied by Dr. Julia K. Butterfield, where most of the validation
metrics we use come from. In this environment, we seek to replicate the
experimental results obtained by Dr. Butterfield of a physical rimless
wheel on a split-belt treadmill. We primarily focused on how the average
steady velocity of the wheel changes in relation to an increasing belt
speed difference. This environment in MuJoCo is able to qualitatively
replicate experimental results, while also remaining quantitatively
accurate. MuJoCo was chosen as the physics simulator due to its accurate
and fast nature, allowing for high-throughput simulations while
providing for a realistic simulation of reality. This allows the user to
easily tinker with and change the system without having to create it
physically and still maintain the ability to observe realistic changes
in dynamics. The repository for this environment is located
[here](https://github.com/PhilipByrn3/dmcontrol_sbt).\
During the creation of this environment many different methods,
simulation parameters, split-belt treadmill and wheel characteristics,
and MuJoCo options were changed and experimented with. Varying these
parameters in different combinations yielded dramatically different
results, primarily from changes in integrator, solver, and contact cone
choices. Other parameters of the wheel and belts were altered initially,
but the primary focus around the physical characteristics of the wheel
and belt was to replicate the experimental setup shown in
\[Butterfield\], so the mass, spoke length, and spoke angle of the wheel
in simulation are identical to the experimental values, likewise with
the 4.9mm offset of the fast belt in the positive z direction. In
simulation the roll and yaw directions of the wheel were eliminated by
constraining the joint to only move within the z-axis and x-axis, as
opposed to the apparatus used experimentally to constrain the direction
of the wheel.

## Replicating the System in MuJoCo

The rimless wheel has two sides with 9 spokes $40\degree$ apart, with
both sides being connected by an axle. In simulation, the wheel has a
mass of 3.53kg, a spoke length of 0.254m, an axle length of 7.6cm, the
sides have an angular offset of $9\degree$, and the spokes which contact
the moving belt have rubber-like attachments to prevent bouncing. These
measurements are identical to those used experimentally except for the
rubber attachments, which were on both sides of the wheel
experimentally. When rubber attachments were used on both sides of the
wheel in simulation, the qualitative properties of the system would
disappear. This can be attributed to the wheel having too much traction
when contacting the stationary belt which results in an excess of
preserved energy in the system, so the average steady velocity of the
wheel increases with large $\Delta$$v_{\mathrm{TM}}$ as opposed to
slowly decreasing with large $\Delta$$v_{\mathrm{TM}}$ .\
In order to replicate the qualitative characteristics of the system, the
wheel should gain energy from the treadmill during contact with the fast
belt and convert it to gravitational potential energy, then passively
convert that potential energy to kinetic energy on contact with the slow
belt. However, at high $\Delta$$v_{\mathrm{TM}}$ the wheel will strike
the slow belt hard enough to lose kinetic energy, which then results in
a slower average velocity. When working in simulation, the primary issue
that was encountered was the \"bouncing issue\", which was also
experimentally observed. This phenomena occurs at larger
$\Delta$$v_{\mathrm{TM}}$ values, where the wheel's transition from
contact with the stationary belt to contact with the fast belt resulted
in the spoke contacting the fast belt to bounce on impact. This problem
introduced many issues, such as the wheel gaining less energy from the
treadmill due to less contact and made observation of the wheel's
dynamic values much harder. The solution for this problem was to attach
rubber-like spheres to the end of the spokes contacting the fast belt.
These spheres were modeled as a spring-mass damper with a large friction
coefficient, resulting in mechanical properties that allow the ends of
the spokes to absorb the high-energy impacts while remaining in contact
with the fast belt.\
To standardize the process for capturing average steady velocity, the
simulations had a maximum of 2000 time steps at 0.01 seconds each to
complete 3 rotations. If the wheel were to complete 3 rotations before
the end of the simulation, the measurements would stop and the
simulation would end. Measurements of the wheel's dynamics would only
begin after a predetermined amount of timesteps, which was chosen to be
50 time steps. This amount of time was chosen since it allows the wheel
to stabilize from the initial impact of contact with the treadmill, and
allowed us to easily capture average steady velocity.

## Determining the Average Steady Velocity of the Rimless Wheel

Average steady velocity was calculated by measuring the final position
of the wheel and dividing by the elapsed time at truncation. This is the
same method used experimentally, but instead of truncating the
experiment when the wheel reaches a predetermined distance, we used the
amount of times the wheel has completed a full rotation since the
predetermined distance was not explicitly stated and because rotation is
an intrinsic property of the wheel.

::: figure*
![image](Figure_1.png){width="\\textwidth"}
:::

# Results

## Simulation and Experimental Result Comparison

Simulations were conducted with a range of belt speeds from 0.15 to 1.1
$\mathrm{m}\cdot\mathrm{s}^{-1}$ , had a maximum time step count of 2000
time steps per simulation at 0.01 seconds each, and used the RK4
integrator, elliptic contact cone, and PGS solver.

# Discussion

' The results of these simulations demonstrate the different parameters
that can be altered to bring simulation results in line with those found
experimentally. The parameters which brought the best results, both
qualitatively and quantitatively, were those of figure 1 and figure 4.
The results of Figure 1 are those of a rimless wheel with rubber ends on
only the spokes which contact the fast belt with no 4.9mm offset. Even
though the experimental setup uses rubber ends on all spokes, the
results of figure 3 show what occurs when this change is implemented.
When it comes to replicating the dynamics of the rimless wheel on a
spllit-belt treadmill, the main constraint that we had to work around
was the bouncing contact that would occur when the fast spoke slams into
the belt and bounces, reducing contact and energy gained. This
constraint involves a delicate balance between reducing bouncing of the
fast spoke, while allowing the slow spoke to dissipate energy. When
these rubber ends are attached to the ends of all spokes, figure 3
results. The wheel is continuously gaining energy from the treadmill and
never dissipating it since there is full contact from the fast spoke and
minimal dissipation from the slow spoke because of the damping effect
from the rubber. With no rubber ends on the spoke and a 4.9mm offset of
the fast belt, figure 5 results, which is both qualitatively and
quantitatively similar to the experimental results. However, when rubber
ends are added to that same simulation, figure 4 results, which is
arguably the best result due to how close the results align to those
found experimentally. However, the primary goal of this environment is
to recreate the qualitative dynamics of the rimless wheel on a
split-belt treadmill, which can be done with either rubber ends attached
to the ends of the fast spoke, or a 4.9mm offset of the fast belt.

## Integrator, Solver, and Contact Cone Choices

\*\*I need some help explaining why RK4 was the best choice.

## The Bouncing Problem and Subsequent Workaround

Initially, the average steady velocity was to be calculated using the
formula
$${}^{S}v_{\mathrm{avg}}=\frac{2l(\sin{\alpha}+sin{\beta})-T_{\mathrm{F}}\Delta v_{\mathrm{TM}}}{T_{\mathrm{S}}+T_{\mathrm{F}}}
\label{svavg}$$ Where ${}^{S}v_{\mathrm{avg}}$ is the average steady
velocity, $l$ is the length of the spoke, $\alpha\ \mathrm{and}\ \beta$
are the internal angles of the spokes relative to each other,
$\Delta$$v_{\mathrm{TM}}$ is the belt speed difference, and $T_S$ and
$T_F$ are the time a spoke is in contact with the slow belt and fast
belt, respectively. The primary issue we encountered when attempting to
solve this equation for the average steady velocity was accurately
determining $T_S$ and $T_F$ while bouncing would occur. Instead of
creating a complicated heuristic to determine the time a spoke is in
contact with either belt, we decided to replicate the method used
experimentally, which was to divide total distance traveled by total
time elapsed. This method produced the most accurate results in the
easiest to implement fashion.

## Keeping Measurements Consistent by Truncating Simulations Early

To reduce the amount of changing variables and standardize the process
for measurement of average steady velocity, simulations would end
depending on the amount of full rotations the wheel has completed on the
treadmill or if the predetermined amount of time steps pass. We allowed
for three full rotations after a small period of time for the wheel to
stabilize itself and station-keep. Rotation count of the wheel was
chosen since it is an intrinsic property of the wheel, as opposed to
distance traveled which is relative to the origin of the world frame.
Tracking rotations in MuJoCo is also simple to implement and consistent
across simulations.

## Limitations

# Conclusion

The primary focus of this paper was to provide an environment for those
interested in split-belt treadmills that is experimentally verified,
easy to use and implement, high fidelity, and fast. In order to
accurately replicate the qualitative dynamics of the rimless wheel on a
split-belt treadmill, the bouncing problem had to be carefully
eliminated through the development of a more complex contact model. This
environment, with the use of either rubber ends on the spoke, and offset
of the belts, or both, was able to successfully recreate the qualitative
and quantitative dynamics of the rimless wheel on a split-belt treadmill
via the reduction of bouncing on the fast belt.

# Acknowledgment {#acknowledgment .unnumbered}

The authors would like to thank\...

::: thebibliography
1

H. Kopka and P. W. Daly, *A Guide to LaTeX*, 3rd ed.Harlow, England:
Addison-Wesley, 1999.
:::

::: IEEEbiography
Michael Shell Biography text here.
:::

::: IEEEbiographynophoto
John Doe Biography text here.
:::

::: IEEEbiographynophoto
Jane Doe Biography text here.
:::

[^1]: P. Byrne was with the Department of Electrical and Computer
    Engineering, Fairfield University, CT 06824 USA e-mail: (see
    http://www.michaelshell.org/contact.html).

[^2]: J. Doe and J. Doe are with Anonymous University.

[^3]: Manuscript received April 19, 2005; revised August 26, 2015.
