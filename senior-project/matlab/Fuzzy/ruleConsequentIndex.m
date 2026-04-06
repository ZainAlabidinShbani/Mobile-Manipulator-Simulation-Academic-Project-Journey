function idx = ruleConsequentIndex(ie, ide, mode)
% input indices: 1..7 => NB NM NS Z PS PM PB
% output indices: 1..5 => VS S M L VL

ae  = abs(ie  - 4);
ade = abs(ide - 4);

switch upper(mode)
    case 'P'
        if ae==0, idx=2;       % S
        elseif ae==1, idx=3;   % M
        elseif ae==2, idx=4;   % L
        else, idx=5;           % VL
        end
        if ade>=2 && idx<5
            idx = idx + 1;
        end

    case 'D'
        if ade==0, idx=2;      % S
        elseif ade==1, idx=3;  % M
        elseif ade==2, idx=4;  % L
        else, idx=5;           % VL
        end

    case 'I'
        if ae==0 && ade==0
            idx=5;             % VL (strong integral at steady state)
        elseif ae<=1 && ade<=1
            idx=4;             % L
        elseif ae<=1 && ade==2
            idx=3;             % M
        else
            idx=1;             % VS (suppress I when far/fast)
        end

    otherwise
        idx=3;
end
end