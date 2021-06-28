function [p, x, y, est] = polynom_fit(x, y, order)
    p = polyfit(x, y, order);
    est = polyval(p, x);
end