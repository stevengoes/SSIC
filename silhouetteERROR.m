function error = silhouetteERROR(p)
% Calculates the error between a silhouette image and a silhouette
% projection of a statistical shape mesh.
% 
% error = silhouetteERROR(p) 
% 
% The vector p consist of translation coordinates [m], 
% rotation angles [rad] and 'b' values [-].
% This cost function consist of the following steps:
%       1) Create the translation vector and rotation matrix from the p
%       vector. Also the b values are defined from teh p vector.
%       2) Create the shape with the following formula: X = meanShape + PC*b
%       3) Project shape X onto an imageplane (or planes) using the imported 
%       projection matrix P. 'n' is the number of vertices.
%       4) Make the image(s) binary and import the silhouette image(s) of the
%       patient
%       5) Compare the images and calculate the error by substracting the
%       projected X from the silhouette and calculate the number of
%       non-zero elements.

global P silhouettes triangleIDs Ncam meanShape PC

tx = p(1);
ty = p(2);
tz = p(3);
rx = p(4);
ry = p(5);
rz = p(6);
b = p(7:length(p));
T = [tx,ty,tz]';
eul = [rz,ry,rx]; % ZYX rotation order (default)
R = eul2rotm(eul);

X = meanShape + PC*b;
X = reshape(X(:),3,[]); %3xn
Xtrans = bsxfun(@plus,(R*X),T); %X = R*X+T; %3xn

ErrorPerCam = ones(1,Ncam);
for cam = 1:Ncam
    
    %calculate pixel locations of the 3D vertices
    res_arm = [Xtrans;ones(1,length(Xtrans))];  %4xn
    RES = P(:,:,cam)*res_arm;
    pImg = round(RES([1 2],:)./RES(3,:));

    % picture of the patient (made in Blender)
    figure(1)
    title('silhouette from image');
    image(silhouettes(:,:,cam),'CDataMapping','scaled');
    axis([0 3260 0 2444])
    set(gca,'position',[0 0 1 1]);
    F1 = getframe(gcf);
    bin1 = rgb2gray(F1.cdata);
    imgbin = double(imbinarize(bin1));
    
    % picture of the projected SSM
    figure(2)
    clf
    title('patch')
    patch( 'Faces', triangleIDs, 'Vertices', pImg', 'FaceColor', [1 1 1], 'EdgeColor', 'none' );
    axis([0 3260 0 2444])
    view(0,-90);
    ax = gca;
    ax.Color = 'k';
    ax.XColor = 'none'; ax.YColor = 'none';
    ax.Position = ax.OuterPosition;
    F2 = getframe(gcf);
    bin2 = rgb2gray(F2.cdata);
    patchbin = double(imbinarize(bin2));
    
    %calculate the error
    if isempty(nonzeros(patchbin))
        ErrorPerCam(cam) = 1;
    else
    error_tmp = length(nonzeros(imgbin-patchbin)); 
    error_tmp = error_tmp / numel(patchbin);
    ErrorPerCam(cam) = error_tmp;
    end
    
end

error = 100*sum(ErrorPerCam)/Ncam; %mean error in [%]

end

