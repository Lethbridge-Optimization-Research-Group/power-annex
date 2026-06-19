## Local search model implementation helpers

The two implementation files each define the three core functions responsible for setting up a model to be optimized using the local-search method.
- set_model_variables 
  : Defines the terms to be considered during optimization, such as ramping costs or power generated. 
- set_model_objective_function 
  : Defines goal of optimization, in our case Min(costs)
- set_model_constraints 
  : Keeps bus power flow and generation feasable given limiting factors like line voltage limits

These are generally going to be called automatically by the primary create_model function. The only difference is that ac models have more variables and constraints to consider, hence a separate file for function redefinition.