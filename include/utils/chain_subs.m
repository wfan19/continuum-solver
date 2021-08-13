% Repeat substitution for a chain of structs
function out = chain_subs(expression, v_structs)

out = expression;

for i = 1 : length(v_structs)
    out = subs(out, v_structs{i});
end

end

