def reward_function(params):

  # Read input parameters
  track_width = params['track_width']
  distance_from_center = params['distance_from_center']
  all_wheels_on_track = params['all_wheels_on_track']
  is_offtrack = params['is_offtrack']

  if distance_from_center <=0.5*track_width:
    reward = 160*(1-distance_from_center/track_width)**2
  else:
    reward = 10

  if not all_wheels_on_track:
    reward= 3	
  if is_offtrack:
    reward = 1e-3

  return float(reward)
