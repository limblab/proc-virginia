function[Rmat] = Rotypr(yw,pt,rl)

Rmat = [cosd(yw).*cosd(pt), -cosd(yw).*sind(pt).*sind(rl)-sind(yw).*cosd(rl), cosd(yw).*sind(pt).*cosd(rl)+sind(yw).*sind(rl);...
    sind(yw).*cosd(pt), sind(yw).*sind(pt).*sind(rl)+cosd(yw).*cosd(rl), sind(yw).*sind(pt).*cosd(rl)-cosd(yw).*sind(rl);...
    -sind(pt), cosd(pt).*sind(rl), cosd(pt).*sind(rl)];

end
