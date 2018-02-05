function[axis,angle] = vec2axang(v1,v2)

angle = acosd((v1*v2')/(norm(v1)*norm(v2)));
axis = cross(v1,v2)/norm(cross(v1,v2));

end