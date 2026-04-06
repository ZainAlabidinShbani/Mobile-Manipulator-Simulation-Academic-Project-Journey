function tau = fuzzy_like(e, ed, Ke, Ked) %#ok<DEFNU>
tau = Ke*tanh(1.15*e) + Ked*tanh(0.55*ed) - 0.60*tanh(0.35*ed);
end