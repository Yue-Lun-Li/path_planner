function ret = llh2enu(roverllh, refllh)

    ref = llh2ecef(refllh);
    rover = llh2ecef(roverllh);
    %delta var = a or b or c =  calcute rover to ref(need to use lla(llh))
    a = rover(1)-ref(1);
    b = rover(2)-ref(2);
    c = rover(3)-ref(3);

    phi= (refllh(1)*pi)/180; %lon
    lam = (refllh(2)*pi)/180; %lat
    sinphi=sin(phi);
    cosphi=cos(phi);
    sinlam=sin(lam);
    coslam=cos(lam);

    x = (-sinlam)*a+(coslam)*b+(0)*c;
    y = (-sinphi*coslam)*a+(-sinphi*sinlam)*b+(cosphi)*c;
    z = (cosphi*coslam)*a+(cosphi*sinlam)*b+(sinphi)*c;
    ret = [x y z];
end