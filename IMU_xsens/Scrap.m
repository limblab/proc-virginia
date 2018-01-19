%% Polyfit
IMU = IMU(1);
ini = find(IMU.stimem>=1,1);
fin = find(IMU.stimem>=IMU.stimem(end)-2,1);
li = IMU.yw(ini:fin);
x = IMU.stimem(ini:fin);
fit = polyfit(x,li,1);
y = zeros(length(x),1);
N = length(fit);
npts = length(x);

for  i = 1:N
    y = y + fit(N-i+1)*x.^(i-1);
end

bin = 3000;
bin = find(IMUs22(1).stimem>=1,1);
nbin = floor(npts/bin);
xbin = [bin:bin:bin*nbin];
yavg = [];

for i = 0:nbin-1
    yavg(i+1) = mean(li(1+i*bin:bin+i*bin));
end

%bpt = find(IMUs22(2).stimem>=7.4,1);
yd = detrend(li,'linear');

figure;
plot(x,li)
hold on
plot(x,y,'r')
plot(x(xbin),yavg,'g')
plot(x,yd,'b')

%% Filtering HF Noise
fLow = 0.1;
fs = 120;
[b,a] = butter(6,fLow*2/fs,'low');
yf = filtfilt(b,a,li);
yfm = li-yf;
yd = detrend(li);

figure;
plot(x,li)
hold on
plot(x,yfm,'r')
%plot(x,y,'g')

%% Wavelets
wname = 'haar';
N = 4;
ord = 14;

[yA,yD] = wdriftcorrect(li,wname,N,ord);

figure;
plot(x,li)
hold on
plot(x,yD,'g')
plot(x,yA,'r')
%plot(x,yD+median(yA),'m')

%% Correct Rsb
for ii = 1:size(IMU,2)
    accv = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
    JA(ii).RAA = vec2Rmat([accv(1) 0 accv(3)],[1 0 0]);
    JA(ii).RFE = vec2Rmat([accv(1:2) 0],[1 0 0]);
    JA(ii).Rsbc = JA(ii).RAA*JA(ii).RFE*JA(ii).Rsb;
end

%% Obtaining calibrated joint angles - using corrected Rsb
joints = {'elb','sho'};
nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        JA(jj).Rgs = ypr2Rmat(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii));
        %JA(jj).Rgs = quat2Rmat(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii)); % Sensor to global matrix
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rmat2ypr(JA(jj).Rgs*JA(jj).Rsbc); % Reconstructed global IMU angles
    end
    
    JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsbc)*(JA(nsho).Rgs*JA(nsho).Rsbc); % Segment to segment matrix
    JA(2).Rbb(:,:,ii) = inv(JA(nback).Rgs*JA(nback).Rsbc)*(JA(nsho).Rgs*JA(nsho).Rsbc); % Segment to segment matrix
    
    for kk = 1:2
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rmat2ypr(JA(kk).Rbb(:,:,ii)); % Reconstructed joint angles
        JA(kk).place = joints{kk};
    end
    
end
