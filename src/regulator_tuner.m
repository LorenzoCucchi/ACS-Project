function R = regulator_tuner(N,name,input,output,type,varargin)

    %{
    HELP
    
    INPUT:
    name: name of the plant
    input: cell array {'','',...} with input variable names (string scalars)
    output: cell array {'','',...} with output variable names (string scalars)
    N: PID1 or PID2
    tf: sought settling time
    varargin: other parameters: c and b
    
    OUTPUT: 
    R: regulator 
    %}
    
    if N==0 % fixed parameters
        if iscell(input)
            R   = pid2(name);
            R.u = input;
            R.y = output;
        else
            R   = pid(name);
            R.u = input;
            R.y = output;
        end
    end
    
    if N==1 % first order tunable parameters
     
        R   = tunablePID(name,type);
        R.u = input;
        R.y = output;
    end
    
    if N == 2 % second order tunable parameters
        
        tf = 0.01; % default
        
        if ~isempty(varargin)
            c = varargin{1};
            b = varargin{2};
        else 
            c = 0;
            b = 1;
        end
        
        R           = tunablePID2(name,type);
        R.c.Value   = c;
        R.c.Free    = false;
        R.b.Value   = b;
        R.b.Free    = false;
        R.Tf.Value  = tf;
        R.Tf.Free   = false;
        R.u         = input;
        R.y         = output;
    end