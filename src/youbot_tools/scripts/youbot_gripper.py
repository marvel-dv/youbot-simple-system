rosservice call /gazebo/set_model_state "
model_state:
  model_name: 'youbot'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.1
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  reference_frame: 'world'
"

rosservice call /gazebo/set_model_state "
model_state:
  model_name: 'block_marker'
  pose:
    position:
      x: 0.8
      y: 0.0
      z: 0.1
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  reference_frame: 'world'
"


