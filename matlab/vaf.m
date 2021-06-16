function v = vaf(y, y_est)

    % Variance Accounted For (VAF) | Percentage value (%)
    %
    % v = vaf(y, y_est)
    %
    % y     : measured output (real)
    % y_est : estimated output
    %
    
    v = var(y - y_est) / var(y) ;
    v = 100 * ( 1 - v );
    
    if ( v < 0 )
        v = 0;
    end

end