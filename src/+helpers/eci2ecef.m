function [Xecef] = eci2ecef(utc,Xeci)
%ECI2ECEF Converts a 2D array from ECI to ECEF.
arguments (Input)
    utc (:,1)
    Xeci (:,3)
end

arguments (Output)
    Xecef (:,3)
end

for i = 1:numel(utc)
    Xecef(i,:) = eci2ecef(utc(i,1),Xeci(i,:));
end