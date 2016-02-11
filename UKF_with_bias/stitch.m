function Final = stitch(imu_ts , R , cam_ts, cam)

fov_a = 100;

ppd = 10;
Final = uint8(zeros(180*ppd, 360*ppd, 3)); %10 pixels per degree
nc = 320;
nr = 240;
pdepth = nr/tand(fov_a/2);
depth = 0.5;


%imcenter = 10*R(:,1, t);
count = 1;
for t = cam_ts
    [~, idx] = min(abs(imu_ts - t));
    fprintf('frame: %d\n', count)
    for j = 1:nr
        for i = 1:nc
            ray = [pdepth; nc/2 - i; nr/2 - j];
            ray_dir = ray;
            %transform ray to get intersection
            XX = R(:, :, idx)*ray_dir;
            %going from intersection coordinate to spherical coordinate
            alpha = atan2(sqrt(XX(1)^2 + XX(2)^2), XX(3));
            cos_beta = XX(1)/depth/sin(alpha);
            sin_beta = XX(2)/depth/sin(alpha);
            beta = atan2(sin_beta, cos_beta);
            alpha = rad2deg(alpha);
            beta = rad2deg(beta);
            
            Finalj = ceil(alpha*ppd);
            Finali = 1800+ceil(beta*ppd);
            
            if Finalj > 1800
                Finalj;
                Finalj = 1800;
            elseif Finalj < 1
                Finalj;
                Finalj = 1;
            end
            
            if Finali > 3600
                Finali;
                Finali = 3600;
            elseif Finali < 1
                Finali;
                Finali = 1;
            end
            
            for k = 1:3
                Final(Finalj, Finali, k) = cam(j, i, k, count);
            end
            
            
        end
    end  
    count = count+1;
    imshow(Final)
end


end