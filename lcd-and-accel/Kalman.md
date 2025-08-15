# Kalman Filter Derivation

```math
\documentclass{article}
\usepackage{amsmath}
\begin{document}

\section*{Simplified Kalman Filter for Angle Estimation}

\textbf{State Vector:}
\[
\mathbf{x}_k =
\begin{bmatrix}
\theta_k \\
\dot{\theta}_k
\end{bmatrix}
\]

\textbf{State Transition Equation:}
\[
\mathbf{x}_k = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_{k-1}
\]
\[
\mathbf{F} =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix}
0 \\
0
\end{bmatrix}
\]

\textbf{Prediction Step:}
\[
\mathbf{x}_k^- = \mathbf{F} \mathbf{x}_{k-1}
\]
\[
\mathbf{P}_k^- = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}
\]

\textbf{Measurement Model:}
\[
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k, \quad
\mathbf{H} =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\]

\textbf{Update Step:}
\[
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}^T \left( \mathbf{H} \mathbf{P}_k^- \mathbf{H}^T + \mathbf{R} \right)^{-1}
\]
\[
\mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k \left( \mathbf{z}_k - \mathbf{H} \mathbf{x}_k^- \right)
\]
\[
\mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{H} \right) \mathbf{P}_k^-
\]

\end{document}
```