function ret = llh2ecef(rover)
    lat = (rover(1)*pi)/180;
    lon = (rover(2)*pi)/180;

    a = 6378137;
    b = 6356752.3142;
    e = sqrt(1-(b/a)*(b/a));
    sinphi = sin(lat);
    cosphi = cos(lat);
    coslam = cos(lon);
    sinlam = sin(lon);
    tan2phi = (tan(lat))*(tan(lat));
    tmp = 1 - e*e;
    tmpden = sqrt( 1 + tmp*tan2phi );
    tmp2 = sqrt(1 - e*e*sinphi*sinphi);

    x = (a*coslam)/tmpden + rover(3)*coslam*cosphi;
    y = (a*sinlam)/tmpden + rover(3)*sinlam*cosphi;
    z = (a*tmp*sinphi)/tmp2 + rover(3)*sinphi;
    ret = [x y z];
end