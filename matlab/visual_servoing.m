
x_res = 1920;
y_res = 1080;
frame_middle = [x_res/2,y_res/2];
face_detect_bbox_rectangle = [x_res/4,y_res/4,x_res/2,y_res/2];
face_detect_bbox = reshape(bbox2points(face_detect_bbox_rectangle)', 1, []);

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector('MinSize',[100,100],'UseROI',true);

% Create the webcam object.
cam = webcam(3);

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

% Init Robot
robot = MyRobot();
robot.move_j(0,-90,0,0);
cw = 0;
pause(2);

while true
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    bbox = faceDetector.step(videoFrameGray,face_detect_bbox_rectangle);

    face_detect_bbox_color = "red";
    
    if ~isempty(bbox)
        bboxPoints = bbox2points(bbox(1, :));

        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        
        bboxPolygon = reshape(bboxPoints', 1, []);
        center = get_center(bboxPolygon);

        if face_inside_bbox(center, frame_middle)
            face_detect_bbox_color = "green";
            dists = get_distances(center, frame_middle);
            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
        else
            face_detect_bbox_color = "red";
        end
    else
        cw = jog_robot_z(cw,robot);
        pause(2);
    end
    videoFrame = insertShape(videoFrame, 'Polygon', face_detect_bbox, 'LineWidth', 3, 'Color',face_detect_bbox_color);

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    if ~runLoop
        break
    end
end

% Clean up.
release(videoPlayer);
release(faceDetector);
robot.disable_motors();


function cw = jog_robot_z(cw,robot)
    if robot.joint_angles(1) == -130
        cw = 1;
    elseif robot.joint_angles(1) == 130
        cw = 0; 
    end
    
    if ~cw
       robot.move_j(robot.joint_angles(1)-10,robot.joint_angles(2),robot.joint_angles(3),robot.joint_angles(4));
    else
        robot.move_j(robot.joint_angles(1)+10,robot.joint_angles(2),robot.joint_angles(3),robot.joint_angles(4));
    end

end

function move_camera(center, frame_middle)
    if frame_middle(1) - center(1) < 0
end


function dists = get_distances(center, frame_middle)
    x_dist = frame_middle(1) - center(1);
    y_dist = frame_middle(2) - center(2);
    dists = [x_dist,y_dist];
end

function center = get_center(bboxPolygon)
    x = floor((bboxPolygon(1) + bboxPolygon(3)) /2);
    y = floor((bboxPolygon(2) + bboxPolygon(4)) /2);
    center = [x,y];

end

function inside = face_inside_bbox(center, frame_middle)
    if abs(frame_middle(1) - center(1)) < frame_middle(1) && abs(frame_middle(2) - center(2)) < frame_middle(2)
        inside = 1;
    else
        inside = 0;
    end
end

