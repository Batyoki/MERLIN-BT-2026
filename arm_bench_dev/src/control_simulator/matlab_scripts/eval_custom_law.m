function u = eval_custom_law(law_text, param_names, param_values, e, dt, integral, derivative, x, r, t)
% EVAL_CUSTOM_LAW Evaluate a free-text control law in MATLAB (or GNU Octave).
% The law should assign to 'u' and may reference e, dt, integral,
% derivative, x, r, t, plus any user-defined parameter names.
% Parameter values are assigned as numbers, not through num2str, so no
% precision is lost (num2str keeps only about 5 significant digits).

    for i = 1:numel(param_names)
        eval([param_names{i} ' = param_values(' num2str(i) ');']);
    end

    u = 0;
    eval(law_text);
end
