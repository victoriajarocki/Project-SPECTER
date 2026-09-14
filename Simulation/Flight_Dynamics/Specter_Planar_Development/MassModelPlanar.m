function m = MassModelPlanar(t,p)

if t <= p.tb
    m = p.mWet-(p.mWet-p.mDry)*(t/p.tb);
else 
    m = p.mDry;
end

end
