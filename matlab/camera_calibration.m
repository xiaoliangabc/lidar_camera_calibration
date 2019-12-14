function camera_calibration(imagePath, resultPath)
    % Define images to process
    imageFiles  = dir(fullfile(imagePath, '*.jpg'));
    imageFileNames = fullfile(imagePath, {imageFiles.name});


    % Detect checkerboards in images
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    imageFileNames = imageFileNames(imagesUsed);
    disp('Detect checkerboard points finish!');

    % Read the first image to obtain image size
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);

    % Generate world coordinates of the corners of the squares
    squareSize = 100;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    disp('Generate checkerboard points finish!');

    % Calibrate the camera
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);
    imageFileNames = imageFileNames(imagesUsed);
    disp('Estimate camera parameters finish!');

    % View reprojection errors
    % h1=figure; showReprojectionErrors(cameraParams);

    % Visualize pattern locations
    % h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

    % Display parameter estimation errors
    % displayErrors(estimationErrors, cameraParams);

    % For example, you can use the calibration data to remove effects of lens distortion.
    % undistortedImage = undistortImage(originalImage, cameraParams);

    % Get file indices
    fileIndices = cell(size(imageFileNames, 2), 1);
    for i = 1 : size(imageFileNames, 2)
        [~, index, ~] = fileparts(string(imageFileNames(i)));
        fileIndices{i} = index;
    end

    % Write transform params to file
    transformParams = [fileIndices num2cell(cameraParams.RotationVectors) num2cell(cameraParams.TranslationVectors./1000)];
    transformParams = cell2table(transformParams, 'VariableNames',{'index' 'rotation_x' 'rotation_y' 'rotation_z' 'translation_x' 'translation_y' 'translation_z'});
    writetable(transformParams, strcat(resultPath, 'camera_chessboard_model.txt'));
    disp('Write chessboard model finish!');

    % Write camera parameters to file
    fid = fopen(strcat(resultPath, 'camera_parameters.txt'),'w');
    % Write intrinsics matrix
    fprintf(fid, '%s', 'intrinsics_matrix');
    intrinsics_vector = reshape(cameraParams.IntrinsicMatrix, 1, 9);
    for i = 1 : size(intrinsics_vector, 2)
        fprintf(fid, ',%f', intrinsics_vector(i));
    end
    fprintf(fid, '\n');
    % Write distortion coefficients
    fprintf(fid, '%s', 'distortion_coefficients');
    distortion_coefficients = [cameraParams.RadialDistortion cameraParams.TangentialDistortion];
    for i = 1 : size(distortion_coefficients, 2)
        fprintf(fid, ',%f', distortion_coefficients(i));
    end
    fprintf(fid, ',%f', 0.0);
    fprintf(fid, '\n');
    % Write image size
    fprintf(fid, '%s', 'image_size');
    for i = 1 : size(cameraParams.ImageSize, 2)
        fprintf(fid, ',%d', cameraParams.ImageSize(i));
    end
    fprintf(fid, '\n');
    disp('Write camera parameters finish!');

    % Write world points to file
    writematrix(worldPoints, strcat(resultPath, 'camera_chessboard_points.txt'));
    disp('Write world points finish!');
end