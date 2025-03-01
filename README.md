# Project Name

⚠️ ​**Note:**​ The source code will be open-sourced upon paper acceptance.  
*(This notice is highlighted with GitHub's warning syntax for visibility)*

## Method Details
### Hyperparameter Configuration
The key hyperparameters and their values used in our method are summarized below:

| Hyperparameter               | Value                      |
|------------------------------|----------------------------|
| Batch size                   | 2048                       |
| Minibatch size               | 64                         |
| Learning rate (α)            | 0.0001                     |
| GAE parameter (λ)            | 0.95                       |
| Discount factor (γ)          | 0.99                       |
| Clipping factor (ε)          | 0.2                        |
| Number of reward models (K)   | 3                          |
| Number of Metrics (M)        | 2 (Lateral Error & Speed Tracking Error) |
| Reward model initial weight (β₀) | 0.05                 |
| Weight decay rate (ρ)         | 0.001                      |
| Smoothing factor (α)         | 0.9                        |
| Inertial coefficients (M₀)   | [1,1,1,1,1,1]             |
| Damping coefficients (D₀)    | [0.1,0.1,0.1,5.5,5.5,5.5] |
| Stiffness coefficients (K₀)  | [500,500,500,500,500,500] |
| Control period (Δt)           | 0.01 s                    |
| PPO decision period          | 1/15 Hz                   |
| Steering radius (R)          | 0.23 m                    |
