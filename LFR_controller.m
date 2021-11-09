function [delR, delL] = LFR_controller(fL,fR,fC, dd)
    if  fL == 1 && fR == 0 && fC == 0
        % Turn right big (large omega)
        delR = dd*2;
        delL = -dd;
    elseif fL == 0 && fR == 1 && fC == 0
        % Turn left big (large omega)
        delR = -dd;
        delL = dd*2;
    elseif  fL == 1 && fR == 1 && fC == 0
        delR = dd*4;
        delL = -dd*4;
    else
        delR = dd*4;
        delL = dd*4;
    end
end

