classdef gl_n
    %gl_n: Base class for Lie algebras
    
    properties(Abstract, Constant)
        dof
        mat_size;
    end
    
    methods
        function obj = gl_n()
        end
    end
    
    methods(Static, Abstract) 
        hat(v_gl_n)
        
        vee(mat_gl_n)
        
        expm(v_gl_n)
    end
end

