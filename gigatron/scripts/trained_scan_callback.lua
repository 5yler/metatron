ros = require 'ros'

require 'torch'
require 'nn'
require 'nngraph'

-- ROS Setup 
ros.init('trained_scan_callback')
nh = ros.NodeHandle()

-- load message specs needed for torch-ros
drive_spec = ros.MsgSpec('gigatron/DriveStamped') 
scan_spec = ros.MsgSpec('sensor_msgs/LaserScan') 

-- loading the neural net model
torch_model_filepath, received = nh:getParamString("torch_model_filepath")

print('loading model '..torch_model_filepath)
model = torch.load(torch_model_filepath)


--  model:cuda()
-- TODO: implement GPU version

-- define output message as global to keep memory allocation fixed
-- this probably doesn't matter because our output is really small
out_command = torch.zeros(1,2):float() 
-- out_command
scan_dim = 360
in_scan = torch.zeros(1,2,scan_dim)

spinner = ros.AsyncSpinner()
spinner:start()

-- subscriber for LaserScan messages
scan_topic, received = nh:getParamString("scan_topic")
scan_topic_queue_size, received = nh:getParamInt("scan_topic_queue_size")
scan_subscriber = nh:subscribe(scan_topic, scan_spec, scan_topic_queue_size)


function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

-- set up drive command publishing
drive_command_topic, received = nh:getParamString("drive_command_topic")
drive_command_queue_size, received = nh:getParamInt("drive_command_queue_size")
drive_command_publisher = nh:advertise(drive_command_topic, drive_spec, drive_command_queue_size, false, connect_cb, disconnect_cb)

-- create messages
drive_msg = ros.Message(drive_spec)

steering_pwm_range = 255
steering_angle_range = 0.8726646   -- [radians] 50 degrees


function angle_from_pwm(angle_pwm)
    angle = (angle_pwm / steering_pwm_range) * steering_angle_range - 0.5 * steering_angle_range;
    return angle
end

print('init complete!')

-- Formatted Subscriber 
scan_subscriber:registerCallback(function(msg, header)
    print('got scan')
    -- print('Header:')
    -- print(header)
    -- print('Message:')
    -- print(msg)
    local totalTime = sys.clock()
    -- local time = sys.clock()
    local data

    scan_dim = 360
    out = torch.zeros(1,2,scan_dim)

    -- print(msg.ranges)

    for i = 1,scan_dim,1 do
        -- print('loading scan '..i)
        -- print(scan[1][i])

        if msg.ranges[i] == math.huge then
            -- check for nan
            -- print('huge!')
            in_scan[1][1][i] = -1   -- range value
        else
            in_scan[1][1][i] = msg.ranges[i] 
        end
        -- if conversion to number fails because value is inf, set -1

        in_scan[1][2][i] = msg.intensities[i]         -- intensity value
    end 

    i = in_scan:reshape(1,360*2)
    -- print(i)

    
    local pred = model:forward(i) --needed size = (102400x7)

    -- Generate the ROS msg
    drive_msg.header = msg.header
    drive_msg.frame_id = 'base_link'

    -- float64 angle
    -- float64 vel_left
    -- float64 vel_right
    drive_msg.drive.vel_left = pred[1][1]
    drive_msg.drive.vel_right = pred[1][1]    drive_msg.drive.angle = angle_from_pwm(pred[1][2])

    -- publish the message
    drive_command_publisher:publish(drive_msg)

    totalTime = sys.clock() - totalTime
    print('total trained scan callback time: '..totalTime..' seconds')
    print('throttle: '..pred[1][1]..' angle '..angle_from_pwm(pred[1][2]))
end)

-- Loop forever
while ros.ok() do
  ros.spinOnce()
end

-- Shutdown functions
subscriber:shutdown()
ros.shutdown()