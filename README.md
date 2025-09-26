# IKFoM Designer Cpp


## Introduction
This is a small C++ library that can be useful for designing an [IKFoM](https://github.com/hku-mars/IKFoM).Its functionality is straightforward: it uses [CasADi](https://web.casadi.org)'s C++ symbolic API to model the system dynamics and leverages CasADi's automatic differentiation capabilities to compute the Jacobian matrices required by the IKFoM toolkit.The library provides tools to define state and measurement functions,construct their Jacobians symbolically, and evaluate them numerically for use in inverse kinematics or filtering frameworks.

This C++ version is **based on the Python version** from [IKFoM-Designer](https://github.com/ErcBunny/IKFoM-Designer.git), adapted to C++ for performance and integration with C++ projects.

 
## Usage
This section explains how to build and run the demo for the IKFoM C++ library.
### 1. Build the project and run the demo

First, create a build directory and run CMake:

```bash
mkdir build
cd build
cmake ..
make
./jacobian_demo
```

The demo will:

Define a simple system using CasADi symbolic variables.

Construct the system dynamics (f) and measurement (h) functions.

Compute the Jacobians using CasADi automatic differentiation.

Print or return the evaluated results.

### 2. Integrate with your own code

You can include the ikfmd headers in your own C++ project:

```cpp
#include "BaseDesigner.hpp"
#include "MX_math.hpp"
```

Then, define your own system and measurement models, and use the library’s automatic differentiation functions to compute Jacobians for:

Inverse kinematics on manifolds (IKFoM)

State estimation or filtering frameworks (e.g., EKF)

This allows seamless integration with custom applications involving robotic kinematics or sensor-based state estimation.

### 3. System Definition and Jacobian Computation

This C++ library provides a **BaseDesigner** class as a template for defining your own system and measurement models. Users implement a derived class (like `demo`) by specifying:

1. **Parameters** $p$  
2. **States** $x$  
3. **State perturbations** $\delta x$  
4. **Inputs** $u$  
5. **Process noises** $w$  
6. **Measurement noises** $v$  

#### a) Dynamics function

$$
f(x, u, w, p) : \mathbb{R}^{n_x} \times \mathbb{R}^{n_u} \times \mathbb{R}^{n_w} \times \mathbb{R}^{n_p} \to \mathbb{R}^{n_x}
$$

Example in `demo`:

$$
f(x,u,w,p) = u + w
$$

#### b) Measurement function

$$
h(x, v, p) : \mathbb{R}^{n_x} \times \mathbb{R}^{n_v} \times \mathbb{R}^{n_p} \to \mathbb{R}^{n_y}
$$

Example in `demo`:

$$
h(x,v,p) = R^\top e_z + v
$$

where $R \in \mathbb{R}^{3 \times 3}$ is the rotation matrix state, $e_z = [0,0,1]^\top$ is the z-axis unit vector, and $v$ is the measurement noise.

#### c) State perturbation function (Boxplus on rotation matrices)

$$
x \boxplus \delta x = R (I_3 + \hat{\delta x})
$$

$$
\hat{\delta x} =
\begin{bmatrix}
0 & -\delta x_3 & \delta x_2 \\
\delta x_3 & 0 & -\delta x_1 \\
-\delta x_2 & \delta x_1 & 0
\end{bmatrix}
$$





