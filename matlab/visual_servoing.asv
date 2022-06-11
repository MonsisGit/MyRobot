
x_res = 640;
y_res = 480;
frame_middle = [x_res/2,y_res/2];
face_detect_bbox_rectangle = [x_res-2,y_res-2,x_res-2,y_res-2];
face_detect_bbox = reshape(bbox2points(face_detect_bbox_rectangle)', 1, []);

% Create the face detector object.
%faceDetector = vision.CascadeObjectDetector('MinSize',[100,100],'UseROI',true);
faceDetector = vision.CascadeObjectDetector('MinSize',[floor(x_res/6),floor(y_res/6)]);

% Create the webcam object.
webcamlist()
cam = webcam(3);

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [200 200 [frameSize(2), frameSize(1)]+30]);

% Init Robot
robot = MyRobot();
assert(robot.is_robot_connected(),"Robot not connected properly");
robot.move_j(0,-90,0,0);
cw = 0;
pause(2);

while true
    try
        videoFrame = snapshot(cam);
        videoFrameGray = rgb2gray(videoFrame);
        %bbox = faceDetector.step(videoFrameGray,face_detect_bbox_rectangle);
        bbox = faceDetector.step(videoFrame);

        if ~isempty(bbox)
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.

            bboxPolygon = reshape(bboxPoints', 1, []);
            center = [bbox(1)+bbox(3)/2,bbox(2)+bbox(4)/2];

            face_detect_bbox_color = "green";
            dists = get_distances(center, frame_middle);
            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3, 'Color',"blue");
            videoFrame = insertShape(videoFrame, 'Circle',[frame_middle,5],'LineWidth', 5, 'Color',"red");
            videoFrame = insertShape(videoFrame, 'Line',[frame_middle,center],'LineWidth', 5, 'Color',"red");
            delta_j1 = (frame_middle(1)-center(1))/40;
            delta_j4 = (frame_middle(2)-center(2))/40;

            try
                robot.move_j(robot.joint_angles(1)+delta_j1,robot.joint_angles(2),robot.joint_angles(3),robot.joint_angles(4));
                robot.move_j(robot.joint_angles(1),robot.joint_angles(2),robot.joint_angles(3),robot.joint_angles(4)+delta_j4);

            catch ME
                disp(ME.message);
            end

        end

        % Display the annotated video frame using the video player object.
        step(videoPlayer, videoFrame);

        % Check whether the video player window has been closed.
        runLoop = isOpen(videoPlayer);
        if ~runLoop
            break
        end
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

